#include <linux/of_device.h>
#include <linux/spi/spidev.h>
#include <asm/uaccess.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <asm/gpio.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <asm/msr.h>
#include "spidev_abas_defs.h"

//Definitions and global variables

#define SPI_CS_MUXSEL 42
#define SPI_MOSI_MUXSEL 43
#define SPI_SCK_MUXSEL 55

#define USS_TRIGGER 27
#define USS_ECHO 14
#define GPIO_2_PU_MUXSEL 0
#define GPIO_2_MUXSEL 31

#define RISING 1
#define FALLING 2

static volatile int echo_received = 0;
static volatile int distance_updated = 0;

struct timespec startd, endd;
struct timespec start, end; //Struct to hold the start and end times of the echo pulse.
struct spi_transfer *myk_tmp; //Struct to hold the data to be transferred along with the required parameters.
static struct task_struct * WorkThread = NULL; //WorkThread thread struct.

unsigned int edge; //Variable to keep track of whether the last 'edge' was RISING or FALLING.
unsigned long long int distance_reading, time_reading;
unsigned int echo_irq = 0;

static int my_work(void *data);
void apply_trigger(void);
/*
 * This supports access to SPI devices using normal userspace I/O calls.
 * Note that while traditional UNIX/POSIX I/O semantics are half duplex,
 * and often mask message boundaries, full SPI support requires full duplex
 * transfers.  There are several kinds of internal message boundaries to
 * handle chipselect management and other protocol options.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/spidevB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */
#define SPIDEV_MAJOR            153 /* assigned */
#define N_SPI_MINORS            32  /* ... up to 256 */

static DECLARE_BITMAP(minors, N_SPI_MINORS);


/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for some settings can cause *lots* of trouble for other
 * devices on a shared bus:
 *
 *  - CS_HIGH ... this device will be active when it shouldn't be
 *  - 3WIRE ... when active, it won't behave as it should
 *  - NO_CS ... there will be no explicit message boundaries; this
 *  is completely incompatible with the shared bus model
 *  - READY ... transfers may proceed when they shouldn't.
 *
 * REVISIT should changing those flags be privileged?
 */
#define SPI_MODE_MASK       (SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
                | SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
                | SPI_NO_CS | SPI_READY)

struct spidev_data
{
    dev_t           devt;
    spinlock_t      spi_lock;
    struct spi_device   *spi;
    struct list_head    device_entry;

    /* buffer is NULL unless this device is open (users > 0) */
    struct mutex        buf_lock;
    unsigned        users;
    u8          *buffer;
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static DEFINE_MUTEX(distance_lock);

static unsigned bufsiz = 4096;
static unsigned min_thresh = 10;
static unsigned max_thresh = 100;

module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

module_param(min_thresh, uint, 0);
MODULE_PARM_DESC(min_thresh, "value in cm for minimum safe distance threshold");

module_param(max_thresh, uint, 0);
MODULE_PARM_DESC(max_thresh, "value in cm for maximum distance");

unsigned int lmin_thresh, lmax_thresh;

//Vatiables for boundary values
unsigned int l1, l2;

static irq_handler_t echo_handler(int irq, void *dev_id, struct pt_regs *regs)
{
    if (echo_received == 0)
    {
        switch (edge)
        {
        case RISING:
            getnstimeofday(&start);
            edge = FALLING;
            irq_set_irq_type(echo_irq, IRQF_TRIGGER_FALLING);
            break;

        case FALLING:
            getnstimeofday(&end);
            edge = RISING;
            irq_set_irq_type(echo_irq, IRQF_TRIGGER_RISING);
            echo_received = 1;
            break;
        }
    }
    return (irq_handler_t)IRQ_HANDLED;
}

/*-------------------------------------------------------------------------*/

/*
 * We can't use the standard synchronous wrappers for file I/O; we
 * need to protect against async removal of the underlying spi_device.
 */
static void spidev_complete(void *arg)
{
    complete(arg);
}

static ssize_t
spidev_sync(struct spidev_data *spidev, struct spi_message *message)
{
    DECLARE_COMPLETION_ONSTACK(done);
    int status;

    message->complete = spidev_complete;
    message->context = &done;

    spin_lock_irq(&spidev->spi_lock);
    if (spidev->spi == NULL)
        status = -ESHUTDOWN;
    else
        status = spi_async(spidev->spi, message);
    spin_unlock_irq(&spidev->spi_lock);

    if (status == 0)
    {
        //printk("\nwaiting...");
        wait_for_completion(&done);
        //printk("\n done waiting...");
        status = message->status;
        if (status == 0)
            status = message->actual_length;
    }
    return status;
}

static inline ssize_t
spidev_sync_write(struct spidev_data *spidev, size_t len)
{
    struct spi_transfer t =
    {
        .tx_buf     = spidev->buffer,
        .len        = len,
    };
    struct spi_message  m;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    return spidev_sync(spidev, &m);
}

static inline ssize_t
spidev_sync_read(struct spidev_data *spidev, size_t len)
{
    struct spi_transfer t =
    {
        .rx_buf     = spidev->buffer,
        .len        = len,
    };
    struct spi_message  m;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    return spidev_sync(spidev, &m);
}

/*-------------------------------------------------------------------------*/

/* Read-only message with current device setup */
static ssize_t
spidev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    struct spidev_data  *spidev;
    ssize_t         status = 0;

    /* chipselect only toggles at start or end of operation */
    if (count > bufsiz)
        return -EMSGSIZE;

    spidev = filp->private_data;

    mutex_lock(&spidev->buf_lock);
    status = spidev_sync_read(spidev, count);
    if (status > 0)
    {
        unsigned long   missing;

        missing = copy_to_user(buf, spidev->buffer, status);
        if (missing == status)
            status = -EFAULT;
        else
            status = status - missing;
    }
    mutex_unlock(&spidev->buf_lock);

    return status;
}

/* Write-only message with current device setup */
static ssize_t
spidev_write(struct file *filp, const char __user *buf,
             size_t count, loff_t *f_pos)
{
    struct spidev_data  *spidev;
    ssize_t         status = 0;
    unsigned long       missing;

    /* chipselect only toggles at start or end of operation */
    if (count > bufsiz)
        return -EMSGSIZE;

    spidev = filp->private_data;

    mutex_lock(&spidev->buf_lock);
    missing = copy_from_user(spidev->buffer, buf, count);
    if (missing == 0)
    {
        status = spidev_sync_write(spidev, count);
    }
    else
        status = -EFAULT;
    mutex_unlock(&spidev->buf_lock);

    return status;
}

static int spidev_message(struct spidev_data *spidev,
                          struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
    struct spi_message  msg;
    struct spi_transfer *k_xfers;
    struct spi_transfer *k_tmp;
    struct spi_ioc_transfer *u_tmp;
    unsigned        n, total;
    u8          *buf;
    int         status = -EFAULT;

    spi_message_init(&msg);
    k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);
    if (k_xfers == NULL)
        return -ENOMEM;

    /* Construct spi_message, copying any tx data to bounce buffer.
     * We walk the array of user-provided transfers, using each one
     * to initialize a kernel version of the same transfer.
     */
    buf = spidev->buffer;
    total = 0;
    for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers;
            n;
            n--, k_tmp++, u_tmp++)
    {
        k_tmp->len = u_tmp->len;

        total += k_tmp->len;
        if (total > bufsiz)
        {
            status = -EMSGSIZE;
            goto done;
        }

        if (u_tmp->rx_buf)
        {
            k_tmp->rx_buf = buf;
            if (!access_ok(VERIFY_WRITE, (u8 __user *)
                           (uintptr_t) u_tmp->rx_buf,
                           u_tmp->len))
                goto done;
        }
        if (u_tmp->tx_buf)
        {
            k_tmp->tx_buf = buf;
            if (copy_from_user(buf, (const u8 __user *)
                               (uintptr_t) u_tmp->tx_buf,
                               u_tmp->len))
                goto done;
        }
        buf += k_tmp->len;

        k_tmp->cs_change = !!u_tmp->cs_change;
        k_tmp->bits_per_word = u_tmp->bits_per_word;
        k_tmp->delay_usecs = u_tmp->delay_usecs;
        k_tmp->speed_hz = u_tmp->speed_hz;
#ifdef VERBOSE
        dev_dbg(&spidev->spi->dev,
                "  xfer len %zd %s%s%s%dbits %u usec %uHz\n",
                u_tmp->len,
                u_tmp->rx_buf ? "rx " : "",
                u_tmp->tx_buf ? "tx " : "",
                u_tmp->cs_change ? "cs " : "",
                u_tmp->bits_per_word ? : spidev->spi->bits_per_word,
                u_tmp->delay_usecs,
                u_tmp->speed_hz ? : spidev->spi->max_speed_hz);
#endif
        spi_message_add_tail(k_tmp, &msg);
    }

    status = spidev_sync(spidev, &msg);
    if (status < 0)
        goto done;

    /* copy any rx data out of bounce buffer */
    buf = spidev->buffer;
    for (n = n_xfers, u_tmp = u_xfers; n; n--, u_tmp++)
    {
        if (u_tmp->rx_buf)
        {
            if (__copy_to_user((u8 __user *)
                               (uintptr_t) u_tmp->rx_buf, buf,
                               u_tmp->len))
            {
                status = -EFAULT;
                goto done;
            }
        }
        buf += u_tmp->len;
    }
    status = total;

done:
    kfree(k_xfers);
    return status;
}


static long
spidev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int         err = 0;
    int         retval = 0;
    struct spidev_data  *spidev;
    struct spi_device   *spi;
    u32         tmp;
    unsigned        n_ioc;
    struct spi_ioc_transfer *ioc;

    /* Check type and command number */
    if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
        return -ENOTTY;

    /* Check access direction once here; don't repeat below.
     * IOC_DIR is from the user perspective, while access_ok is
     * from the kernel perspective; so they look reversed.
     */
    if (_IOC_DIR(cmd) & _IOC_READ)
        err = !access_ok(VERIFY_WRITE,
                         (void __user *)arg, _IOC_SIZE(cmd));
    if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
        err = !access_ok(VERIFY_READ,
                         (void __user *)arg, _IOC_SIZE(cmd));
    if (err)
        return -EFAULT;

    /* guard against device removal before, or while,
     * we issue this ioctl.
     */
    spidev = filp->private_data;
    spin_lock_irq(&spidev->spi_lock);
    spi = spi_dev_get(spidev->spi);
    spin_unlock_irq(&spidev->spi_lock);

    if (spi == NULL)
        return -ESHUTDOWN;

    /* use the buffer lock here for triple duty:
     *  - prevent I/O (from us) so calling spi_setup() is safe;
     *  - prevent concurrent SPI_IOC_WR_* from morphing
     *    data fields while SPI_IOC_RD_* reads them;
     *  - SPI_IOC_MESSAGE needs the buffer locked "normally".
     */
    mutex_lock(&spidev->buf_lock);

    switch (cmd)
    {
    case SPI_SET_MIN_THRESH:
        retval = __get_user(tmp, (u32 __user *)arg);
        if (retval == 0)
        {
           // printk("\nRequest to set the minimum threshold.");
            //If either min_thresh or max_thresh are outside hardware capability, reset them.
            
            if (tmp < 2)
            {
                retval = -1;
            }

            if (tmp >= lmax_thresh)
            {
                retval = -1;
            }

            if (retval != -1)
            {
                lmin_thresh = tmp;
                //Caculate the boundary values.
                l1 = ((lmax_thresh - lmin_thresh)/3) + lmin_thresh;
                l2 = l1 + ((lmax_thresh - lmin_thresh)/3);
            }
            
        }
        
        break;
    case SPI_SET_MAX_THRESH:
        retval = __get_user(tmp, (u32 __user *)arg);
        if (retval == 0)
        {
            //printk("\nRequest to set the minimum threshold.");
            //If either min_thresh or max_thresh are outside hardware capability, reset them.
            lmax_thresh = tmp;

            if (tmp > 400)
            {
                retval = -1;
            }

            if (lmin_thresh >= tmp)
            {
                retval = -1;
            }

            if (retval != -1)
            {
                lmax_thresh = tmp;
                //Caculate the boundary values.
                l1 = ((lmax_thresh - lmin_thresh)/3) + lmin_thresh;
                l2 = l1 + ((lmax_thresh - lmin_thresh)/3);
            }
        }
        break;
    /* read requests */
    case SPI_IOC_RD_MODE:
        retval = __put_user(spi->mode & SPI_MODE_MASK,
                            (__u8 __user *)arg);
        break;
    case SPI_IOC_RD_LSB_FIRST:
        retval = __put_user((spi->mode & SPI_LSB_FIRST) ?  1 : 0,
                            (__u8 __user *)arg);
        break;
    case SPI_IOC_RD_BITS_PER_WORD:
        retval = __put_user(spi->bits_per_word, (__u8 __user *)arg);
        break;
    case SPI_IOC_RD_MAX_SPEED_HZ:
        retval = __put_user(spi->max_speed_hz, (__u32 __user *)arg);
        break;

    /* write requests */
    case SPI_IOC_WR_MODE:
        retval = __get_user(tmp, (u8 __user *)arg);
        if (retval == 0)
        {
            u8  save = spi->mode;

            if (tmp & ~SPI_MODE_MASK)
            {
                retval = -EINVAL;
                break;
            }

            tmp |= spi->mode & ~SPI_MODE_MASK;
            spi->mode = (u8)tmp;
            retval = spi_setup(spi);
            if (retval < 0)
                spi->mode = save;
            else
                dev_dbg(&spi->dev, "spi mode %02x\n", tmp);
        }
        break;
    case SPI_IOC_WR_LSB_FIRST:
        retval = __get_user(tmp, (__u8 __user *)arg);
        if (retval == 0)
        {
            u8  save = spi->mode;

            if (tmp)
                spi->mode |= SPI_LSB_FIRST;
            else
                spi->mode &= ~SPI_LSB_FIRST;
            retval = spi_setup(spi);
            if (retval < 0)
                spi->mode = save;
            else
                dev_dbg(&spi->dev, "%csb first\n",
                        tmp ? 'l' : 'm');
        }
        break;
    case SPI_IOC_WR_BITS_PER_WORD:
        retval = __get_user(tmp, (__u8 __user *)arg);
        if (retval == 0)
        {
            u8  save = spi->bits_per_word;

            spi->bits_per_word = tmp;
            retval = spi_setup(spi);
            if (retval < 0)
                spi->bits_per_word = save;
            else
                dev_dbg(&spi->dev, "%d bits per word\n", tmp);
        }
        break;
    case SPI_IOC_WR_MAX_SPEED_HZ:
        retval = __get_user(tmp, (__u32 __user *)arg);
        if (retval == 0)
        {
            u32 save = spi->max_speed_hz;

            spi->max_speed_hz = tmp;
            retval = spi_setup(spi);
            if (retval < 0)
                spi->max_speed_hz = save;
            else
                dev_dbg(&spi->dev, "%d Hz (max)\n", tmp);
        }
        break;

    default:
        /* segmented and/or full-duplex I/O request */
        if (_IOC_NR(cmd) != _IOC_NR(SPI_IOC_MESSAGE(0))
                || _IOC_DIR(cmd) != _IOC_WRITE)
        {
            retval = -ENOTTY;
            break;
        }

        tmp = _IOC_SIZE(cmd);
        if ((tmp % sizeof(struct spi_ioc_transfer)) != 0)
        {
            retval = -EINVAL;
            break;
        }
        n_ioc = tmp / sizeof(struct spi_ioc_transfer);
        if (n_ioc == 0)
            break;

        /* copy into scratch area */
        ioc = kmalloc(tmp, GFP_KERNEL);
        if (!ioc)
        {
            retval = -ENOMEM;
            break;
        }
        if (__copy_from_user(ioc, (void __user *)arg, tmp))
        {
            kfree(ioc);
            retval = -EFAULT;
            break;
        }

        /* translate to spi_message, execute */
        retval = spidev_message(spidev, ioc, n_ioc);
        kfree(ioc);
        break;
    }

    mutex_unlock(&spidev->buf_lock);
    spi_dev_put(spi);
    return retval;
}


static long
my_spidev_ioctl(struct spidev_data  *spidev, unsigned int cmd, unsigned long val)
{
    int         retval = 0;
    struct spi_device   *spi;
    u32         tmp = 0;

    /* Check type and command number */
    if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
        return -ENOTTY;

    spin_lock_irq(&spidev->spi_lock);
    spi = spi_dev_get(spidev->spi);
    spin_unlock_irq(&spidev->spi_lock);

    if (spi == NULL)
        return -ESHUTDOWN;

    /* use the buffer lock here for triple duty:
     *  - prevent I/O (from us) so calling spi_setup() is safe;
     *  - prevent concurrent SPI_IOC_WR_* from morphing
     *    data fields while SPI_IOC_RD_* reads them;
     *  - SPI_IOC_MESSAGE needs the buffer locked "normally".
     */
    mutex_lock(&spidev->buf_lock);

    switch (cmd)
    {

    /* write requests */
    case SPI_IOC_WR_MODE:
        retval = 0;//__get_user(tmp, (u8 __user *)arg);
        if (retval == 0)
        {
            u8  save = spi->mode;
            tmp = val;
            //printk("\n write mode value is: %d", tmp);
            if (tmp & ~SPI_MODE_MASK)
            {
                retval = -EINVAL;
                break;
            }

            tmp |= spi->mode & ~SPI_MODE_MASK;
            spi->mode = (u8)tmp;
            retval = spi_setup(spi);
            if (retval < 0)
                spi->mode = save;
            else
                dev_dbg(&spi->dev, "spi mode %02x\n", tmp);
        }
        break;
    case SPI_IOC_WR_LSB_FIRST:
        retval = 0;//__get_user(tmp, (__u8 __user *)arg);
        if (retval == 0)
        {
            u8  save = spi->mode;
            tmp = val;
            //printk("\n lsb first val is: %d", tmp);
            if (tmp)
                spi->mode |= SPI_LSB_FIRST;
            else
                spi->mode &= ~SPI_LSB_FIRST;
            retval = spi_setup(spi);
            if (retval < 0)
                spi->mode = save;
            else
                dev_dbg(&spi->dev, "%csb first\n",
                        tmp ? 'l' : 'm');
        }
        break;
    case SPI_IOC_WR_BITS_PER_WORD:
        retval = 0;//__get_user(tmp, (__u8 __user *)arg);
        if (retval == 0)
        {
            u8  save = spi->bits_per_word;
            tmp = val;
            spi->bits_per_word = tmp;
            //printk("\n BPW value is: %d", tmp);
            retval = spi_setup(spi);
            if (retval < 0)
                spi->bits_per_word = save;
            else
                dev_dbg(&spi->dev, "%d bits per word\n", tmp);
        }
        break;
    case SPI_IOC_WR_MAX_SPEED_HZ:
        retval = 0;//__get_user(tmp, (__u32 __user *)arg);
        if (retval == 0)
        {

            u32 save = spi->max_speed_hz;
            tmp = val;
            //printk("\n Max speed is: %d", tmp);
            spi->max_speed_hz = tmp;
            retval = spi_setup(spi);
            if (retval < 0)
                spi->max_speed_hz = save;
            else
                dev_dbg(&spi->dev, "%d Hz (max)\n", tmp);
        }
        break;
    }

    mutex_unlock(&spidev->buf_lock);
    spi_dev_put(spi);
    return retval;
}


#ifdef CONFIG_COMPAT
static long
spidev_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return spidev_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define spidev_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int spidev_open(struct inode *inode, struct file *filp)
{
    struct spidev_data  *spidev;
    int         status = -ENXIO;

    mutex_lock(&device_list_lock);

    list_for_each_entry(spidev, &device_list, device_entry)
    {
        if (spidev->devt == inode->i_rdev)
        {
            status = 0;
            break;
        }
    }
    if (status == 0)
    {
        if (!spidev->buffer)
        {
            spidev->buffer = kmalloc(bufsiz, GFP_KERNEL);
            if (!spidev->buffer)
            {
                dev_dbg(&spidev->spi->dev, "open/ENOMEM\n");
                status = -ENOMEM;
            }
        }
        if (status == 0)
        {
            spidev->users++;
            filp->private_data = spidev;
            nonseekable_open(inode, filp);
        }
    }
    else
        pr_debug("spidev: nothing for minor %d\n", iminor(inode));

    mutex_unlock(&device_list_lock);
    return status;
}

static int spidev_release(struct inode *inode, struct file *filp)
{
    struct spidev_data  *spidev;
    int         status = 0;

    mutex_lock(&device_list_lock);
    spidev = filp->private_data;
    filp->private_data = NULL;

    /* last close? */
    spidev->users--;
    if (!spidev->users)
    {
        int     dofree;

        kfree(spidev->buffer);
        spidev->buffer = NULL;

        /* ... after we unbound from the underlying device? */
        spin_lock_irq(&spidev->spi_lock);
        dofree = (spidev->spi == NULL);
        spin_unlock_irq(&spidev->spi_lock);

        if (dofree)
            kfree(spidev);
    }
    mutex_unlock(&device_list_lock);

    return status;
}


static const struct file_operations spidev_fops =
{
    .owner =    THIS_MODULE,
    /* REVISIT switch to aio primitives, so that userspace
     * gets more complete API coverage.  It'll simplify things
     * too, except for the locking.
     */
    .write =    spidev_write,
    .read =     spidev_read,
    .unlocked_ioctl = spidev_ioctl,
    .compat_ioctl = spidev_compat_ioctl,
    .open =     spidev_open,
    .release =  spidev_release,
    .llseek =   no_llseek,
};

/*-------------------------------------------------------------------------*/

/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *spidev_class;

/*-------------------------------------------------------------------------*/

static int spidev_probe(struct spi_device *spi)
{
    struct spidev_data  *spidev;
    int         status;
    unsigned long       minor;

    int ret;

    /* Allocate driver data */
    spidev = kzalloc(sizeof(*spidev), GFP_KERNEL);
    if (!spidev)
        return -ENOMEM;

    /* Initialize the driver data */
    spidev->spi = spi;
    spin_lock_init(&spidev->spi_lock);
    mutex_init(&spidev->buf_lock);

    INIT_LIST_HEAD(&spidev->device_entry);

    /* If we can allocate a minor number, hook up this device.
     * Reusing minors is fine so long as udev or mdev is working.
     */
    mutex_lock(&device_list_lock);
    minor = find_first_zero_bit(minors, N_SPI_MINORS);
    if (minor < N_SPI_MINORS)
    {
        struct device *dev;

        spidev->devt = MKDEV(SPIDEV_MAJOR, minor);
        dev = device_create(spidev_class, &spi->dev, spidev->devt,
                            spidev, "spidev%d.%d",
                            spi->master->bus_num, spi->chip_select);
        status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
    }
    else
    {
        dev_dbg(&spi->dev, "no minor number available!\n");
        status = -ENODEV;
    }
    if (status == 0)
    {
        set_bit(minor, minors);
        list_add(&spidev->device_entry, &device_list);
    }
    mutex_unlock(&device_list_lock);

    if (status == 0)
        spi_set_drvdata(spi, spidev);
    else
        kfree(spidev);

    //SPI GPIO setup
    ret = gpio_request(SPI_CS_MUXSEL, "CS_pin");
    if(ret < 0)
    {
        printk("Error Requesting Pin SPI_CS_MUXSEL.\n");
        return -1;
    }
    ret = gpio_direction_output(SPI_CS_MUXSEL, 0);
    if(ret < 0)
    {
        printk("Error Setting Pin SPI_CS_MUXSEL Output.\n");
    }

    ret = gpio_request(SPI_MOSI_MUXSEL, "MOSI_pin");
    if(ret < 0)
    {
        printk("Error Requesting Pin SPI_MOSI_MUXSEL.\n");
        return -1;
    }
    ret = gpio_direction_output(SPI_MOSI_MUXSEL, 0);
    if(ret < 0)
    {
        printk("Error Setting Pin SPI_MOSI_MUXSEL Output.\n");
    }

    ret = gpio_request(SPI_SCK_MUXSEL, "CLK_pin");
    if(ret < 0)
    {
        printk("Error Requesting Pin SPI_SCK_MUXSEL.\n");
        return -1;
    }
    ret = gpio_direction_output(SPI_SCK_MUXSEL, 0);
    if(ret < 0)
    {
        printk("Error Setting Pin SPI_SCK_MUXSEL Output.\n");
    }

    gpio_set_value_cansleep(SPI_CS_MUXSEL, 0);
    gpio_set_value_cansleep(SPI_MOSI_MUXSEL, 0);
    gpio_set_value_cansleep(SPI_SCK_MUXSEL, 0);

    //Ultrasonic sensor GPIO setup.
    ret = gpio_request(USS_TRIGGER, "gpio_trigger");
    if(ret < 0)
    {
        printk("Error Requesting Trigger Pin.\n");

        return -1;
    }
    ret = gpio_direction_output(USS_TRIGGER, 0);
    if(ret < 0)
    {
        printk("Error Setting Trigger Pin Output.\n");
    }

    ret = gpio_request(GPIO_2_MUXSEL, "gpio_2_Mux");
    if(ret < 0)
    {
        printk("Error Requesting GPIO2_Mux.\n");
        return -1;
    }
    ret = gpio_direction_output(GPIO_2_MUXSEL, 0);
    if(ret < 0)
    {
        printk("Error Setting GPIO_2_MUXSEL output.\n");
    }

    ret = gpio_request(GPIO_2_PU_MUXSEL, "gpio_2_puMux");
    if(ret < 0)
    {
        printk("Error Requesting GPIO2_puMux.\n");

        return -1;
    }
    ret = gpio_direction_output(GPIO_2_PU_MUXSEL, 0);
    if(ret < 0)
    {
        printk("Error Setting GPIO_2_puMux output.\n");
    }

    ret = gpio_request(USS_ECHO, "gpio_echo");
    if(ret < 0)
    {
        printk("Error Requesting echo Pin.\n");
        return -1;
    }
    ret = gpio_direction_input(USS_ECHO);
    if(ret < 0)
    {
        printk("Error Setting echo Pin Input.\n");
    }

    gpio_set_value_cansleep(USS_TRIGGER, 0); // Initialize output to off.
    gpio_set_value_cansleep(GPIO_2_MUXSEL, 0); // Set GPIO 2 Mux to 0.
    gpio_set_value_cansleep(GPIO_2_PU_MUXSEL, 0); // Set GPIO 2 pullup Mux to 0.

    //Setup the interrupt for the echo signal.
    echo_irq = gpio_to_irq(USS_ECHO);

    edge = RISING;
    ret = request_irq(echo_irq, (irq_handler_t)echo_handler, IRQF_TRIGGER_RISING, "echo_Dev", (void *)(echo_irq));
    if(ret < 0)
    {
        printk("Error requesting IRQ: %d\n", ret);
    }

    //Allocate memory.
    myk_tmp = kcalloc(1, sizeof(*myk_tmp), GFP_KERNEL);

    //Start the 'main' thread.
    WorkThread = kthread_run(my_work, (void *)spi,"work_test");

    return status;
}


static int spidev_remove(struct spi_device *spi)
{
    struct spidev_data  *spidev = spi_get_drvdata(spi);

    /* make sure ops on existing fds can abort cleanly */
    spin_lock_irq(&spidev->spi_lock);
    spidev->spi = NULL;
    spi_set_drvdata(spi, NULL);
    spin_unlock_irq(&spidev->spi_lock);

    /* prevent new opens */
    mutex_lock(&device_list_lock);
    list_del(&spidev->device_entry);
    device_destroy(spidev_class, spidev->devt);
    clear_bit(MINOR(spidev->devt), minors);
    if (spidev->users == 0)
        kfree(spidev);
    mutex_unlock(&device_list_lock);

    return 0;
}


static const struct of_device_id spidev_dt_ids[] =
{
    { .compatible = "rohm,dh2228fv" },
    {},
};


MODULE_DEVICE_TABLE(of, spidev_dt_ids);


static struct spi_driver spidev_spi_driver =
{
    .driver = {
        .name =     "spidev",
        .owner =    THIS_MODULE,
        .of_match_table = of_match_ptr(spidev_dt_ids),
    },
    .probe =    spidev_probe,
    .remove =   spidev_remove,

    /* NOTE:  suspend/resume methods are not necessary here.
     * We don't do anything except pass the requests to/from
     * the underlying controller.  The refrigerator handles
     * most issues; the controller driver handles the rest.
     */
};

/*-------------------------------------------------------------------------*/

static int __init spidev_init(void)
{
    int status;

    /* Claim our 256 reserved device numbers.  Then register a class
     * that will key udev/mdev to add/remove /dev nodes.  Last, register
     * the driver which manages those device numbers.
     */
    BUILD_BUG_ON(N_SPI_MINORS > 256);
    status = register_chrdev(SPIDEV_MAJOR, "spi", &spidev_fops);
    if (status < 0)
        return status;

    spidev_class = class_create(THIS_MODULE, "spidev");
    if (IS_ERR(spidev_class))
    {
        unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
        return PTR_ERR(spidev_class);
    }

    status = spi_register_driver(&spidev_spi_driver);
    if (status < 0)
    {
        class_destroy(spidev_class);
        unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
    }
    return status;
}

void write_to_display(struct spidev_data *spidev, struct spi_transfer *k_tmp)
{
    struct spi_message  msg;

    spi_message_init(&msg);
    spi_message_add_tail(k_tmp, &msg);
    spidev_sync(spidev, &msg);
    return;
}

void apply_trigger(void)
{
    echo_received = 0;
    gpio_set_value_cansleep(USS_TRIGGER, 0);
    udelay(2);
    gpio_set_value_cansleep(USS_TRIGGER, 1);
    udelay(10);
    gpio_set_value_cansleep(USS_TRIGGER, 0);
}

static int my_work(void *data)
{
    //Initialize a 2D array that holds all the bitmap images.
    unsigned char bmap_data[6][8]=
    {
        {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},//All blank
        {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff},//All filled
        {0x00,0xff,0xff,0xff,0xff,0xff,0xff,0x00},
        {0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00},
        {0x00,0x00,0x00,0xff,0xff,0x00,0x00,0x00},
        {0x81,0xc3,0xa5,0x18,0x18,0xa5,0xc3,0x81},//X
    };

    struct spi_device *spi = (struct spi_device*) data;
    struct spidev_data  *spidev = spi_get_drvdata(spi);

    //Other variables for functionality and setting up the SPI.
    int i, old_val = 0, toggle = 0;
    char writeBuf[2];
    unsigned int mytemp = 0;

    //Maintain a record of the previous state so that we don't end up sending the same bitmap data again.
    int prev_state = 0, curr_state = 0;

    lmin_thresh = min_thresh;
    lmax_thresh = max_thresh;

    //-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-

    //Set SPI_IOC_WR_MODE to 0
    mytemp |= SPI_MODE_0;
    my_spidev_ioctl(spidev, SPI_IOC_WR_MODE, mytemp);

    //Set SPI_IOC_WR_LSB_FIRST to 0
    my_spidev_ioctl(spidev, SPI_IOC_WR_LSB_FIRST, mytemp);

    //Set SPI_IOC_WR_BITS_PER_WORD to 8
    mytemp = 8;
    my_spidev_ioctl(spidev, SPI_IOC_WR_BITS_PER_WORD, mytemp);

    //Set SPI_IOC_WR_MAX_SPEED_HZ to max
    mytemp = 0;
    my_spidev_ioctl(spidev, SPI_IOC_WR_MAX_SPEED_HZ, mytemp);

    //If either min_thresh or max_thresh are outside hardware capability, reset them.
    if (min_thresh < 2)
        lmin_thresh = 2;

    if (max_thresh > 400)
        lmax_thresh = 400;

    if (min_thresh >= max_thresh)
    {
        lmin_thresh = 10;
        lmax_thresh = 100;
    }

    //Caculate the boundary values.
    l1 = ((lmax_thresh - lmin_thresh)/3) + lmin_thresh;
    l2 = l1 + ((lmax_thresh - lmin_thresh)/3);

    //Setup the SPI transfer buffer and set the display parameters.
    myk_tmp->tx_buf = writeBuf;
    myk_tmp->rx_buf = 0;
    myk_tmp->len = 2;
    myk_tmp->delay_usecs = 0;
    myk_tmp->bits_per_word = 8;
    myk_tmp->speed_hz = 25000000;

    //Set the decoding mode as BCD.
    writeBuf[0] = 0x09;
    writeBuf[1] = 0x00;
    write_to_display(spidev, myk_tmp);

    //Set the brightness.
    writeBuf[0] = 0x0A;
    writeBuf[1] = 0x03;
    write_to_display(spidev, myk_tmp);

    //Set the scan limit to 8 LEDs.
    writeBuf[0] = 0x0B;
    writeBuf[1] = 0x07;
    write_to_display(spidev, myk_tmp);

    //Set to normal mode.
    writeBuf[0] = 0x0C;
    writeBuf[1] = 0x01;
    write_to_display(spidev, myk_tmp);

    //Flash the display twice.
    for(i = 0; i < 2; i++)
    {
        //printk("\nBlink!");
        writeBuf[0] = 0x0F;
        writeBuf[1] = 0x01;
        write_to_display(spidev, myk_tmp);

        msleep(250);

        writeBuf[0] = 0x0F;
        writeBuf[1] = 0x00;
        write_to_display(spidev, myk_tmp);

        msleep(250);
    }

    //Blank the display
    for(i = 1; i < 9; i++)
    {
        // Setup a Write Transfer
        writeBuf[0] = i; //Select row
        writeBuf[1] = bmap_data[0][i-1]; //Write row data
        write_to_display(spidev, myk_tmp);
    }

    while(!kthread_should_stop())
    {
        if (echo_received == 1 && distance_updated == 0)
        {
            //If an echo pulse has been received, calculate the distance.
            distance_reading = (end.tv_sec - start.tv_sec) + end.tv_nsec - start.tv_nsec;

            //Convert time to microseconds from nanoseconds
            do_div(distance_reading, 1000);

            //Convert time in microseconds to distance in centimeters.
            do_div(distance_reading, 58);

            //Display the distance and the pulse width.
            /*
            printk("\nDistance: %llu cm - Pulse width: %llu us - Min Thresh: %d, Max Thresh: %d",
             (long long unsigned int) distance_reading,
              distance_reading*58,
              lmin_thresh,
              lmax_thresh);
            */

            //Set the distance updated flag
            if (distance_reading != old_val || distance_reading <= min_thresh)
                distance_updated = 1;

            //Update the old_value;
            old_val = distance_reading;
        }

        //Only evaluate the new state if there was a change in the distance reading.
        if (distance_updated == 1)
        {
            //Evaluate the current state based upon the current distance reading.
            if (distance_reading < lmin_thresh && toggle == 0)
                curr_state = 5;
            else if (distance_reading < lmin_thresh && toggle == 1)
                curr_state = 0;
            else if (distance_reading >= lmin_thresh && distance_reading < l1)
                curr_state = 4;
            else if (distance_reading >= l1 && distance_reading < l2)
                curr_state = 3;
            else if (distance_reading >= l2 && distance_reading < lmax_thresh)
                curr_state = 2;
            else if (distance_reading >= max_thresh)
                curr_state = 1;

            //Only update the display if the current state is different from the previous state
            if (prev_state != curr_state)
            {
                switch (curr_state)
                {
                case 0:
                    for(i = 1; i < 9; i++)
                    {
                        writeBuf[0] = i; //Column
                        writeBuf[1] = bmap_data[0][i-1]; //Data
                        write_to_display(spidev, myk_tmp);
                    }
                    toggle = 0;
                    break;

                case 1:
                    for(i = 1; i < 9; i++)
                    {
                        writeBuf[0] = i; //Column
                        writeBuf[1] = bmap_data[1][i-1]; //Data
                        write_to_display(spidev, myk_tmp);
                    }
                    break;

                case 2:
                    for(i = 1; i < 9; i++)
                    {
                        writeBuf[0] = i; //Column
                        writeBuf[1] = bmap_data[2][i-1]; //Data
                        write_to_display(spidev, myk_tmp);
                    }
                    break;

                case 3:
                    for(i = 1; i < 9; i++)
                    {
                        writeBuf[0] = i; //Column
                        writeBuf[1] = bmap_data[3][i-1]; //Data
                        write_to_display(spidev, myk_tmp);
                    }
                    break;

                case 4:
                    for(i = 1; i < 9; i++)
                    {
                        writeBuf[0] = i; //Column
                        writeBuf[1] = bmap_data[4][i-1]; //Data
                        write_to_display(spidev, myk_tmp);
                    }
                    break;

                case 5:
                    for(i = 1; i < 9; i++)
                    {
                        writeBuf[0] = i; //Column
                        writeBuf[1] = bmap_data[5][i-1]; //Data
                        write_to_display(spidev, myk_tmp);
                    }
                    toggle = 1;
                    break;
                }
            }
            prev_state = curr_state;

            distance_updated = 0;
        }
        apply_trigger();
        mdelay(60);
    }

    return 0;
}

module_init(spidev_init);

static void __exit spidev_exit(void)
{
    //Kill the thread.
    if(WorkThread)
    {
        kthread_stop(WorkThread);
    }

    //Free the IRQ.
    free_irq(echo_irq, (void *)(echo_irq));

    //Release all the GPIOs.
    gpio_free(USS_TRIGGER);
    gpio_free(GPIO_2_MUXSEL);
    gpio_free(GPIO_2_PU_MUXSEL);
    gpio_free(USS_ECHO);

    gpio_free(SPI_CS_MUXSEL);
    gpio_free(SPI_MOSI_MUXSEL);
    gpio_free(SPI_SCK_MUXSEL);

    //Free memory
    kfree(myk_tmp);

    //SPI clean up.
    spi_unregister_driver(&spidev_spi_driver);
    class_destroy(spidev_class);
    unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);

}
module_exit(spidev_exit);

MODULE_AUTHOR("Andrea Paterniani, <a.paterniani@swapp-eng.it> / Adil Faqah");
MODULE_DESCRIPTION("User mode SPI device interface / ABAS");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:spidev");
