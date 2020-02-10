/*
 *  Microchip QProx™  QT60248 QMatrix™ 
 *
 *  Authors: Gabor Wunderlich <wunderlichg.@t-online.hu>
 *
 *  Base on QT60248 driver by:
 *  Gabor Wunderlich <wunderlichg@t-online.hu>
 *  Copyright (C) 2009
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/input/qt60248.h>
#include <linux/workqueue.h>

#define QT_DEBUG
#ifdef QT_DEBUG
#define DBG(x...) printk(x)
#else
#define DBG(x...) do {} while (0)
#endif


static const unsigned short qt60248_key2code[] = {
	KEY_CAPSLOCK, KEY_RIGHT, KEY_ENTER,
	KEY_3, KEY_9, KEY_6,
	KEY_F1, KEY_F2, KEY_F3,
	KEY_LEFT, KEY_BACKSPACE,
	KEY_1, KEY_7, KEY_4,
	KEY_RESERVED, KEY_RESERVED,
	KEY_UP, KEY_DOWN,
	KEY_0, KEY_2, KEY_8, KEY_5
};


typedef enum
{
    QT_NULL         =   0x00,
    QT_LD_SETUP     =   0x01,
    QT_CAL_ALL      =   0x03,
    QT_RESET        =   0x04,
    QT_GEN_STATUS   =   0x05,
    QT_REP_1STKEY   =   0x06,
    QT_REP_ALLKEY   =   0x07,
    QT_ERR_ALLKEY   =   0x0B,
    QT_FMEA_STAT    =   0x0C,
    QT_EEPROM_CRC   =   0x0E,
    QT_LAST_CMD     =   0x0F,
    QT_CAL_KEY      =   0xC0,
} t_qtcmd;

/*** 1st key status bits *****************/
#define MORE_THAN1  0x80
#define ANY_ERROR       0x40
/*** general status bits *****************/
#define COMM_ERR    0x40
#define FMEA_ERR    0x20
#define SYNC_ERR    0x08
#define CAL_ERR     0x04
#define IN_CAL      0x02
#define IN_DET      0x01

#define KEY_MASK    0x003F3FFF
#define WAIT_MS_AFTER_POLL	20

typedef enum
{
    QTSTM_CHECK,
    QTSTM_CHECK_RDY,
    QTSTM_CHK_SETUPCRC,
    QTSTM_CHK_SETUPCRC_RDY,
    QTSTM_RESET,
    QTSTM_POLL_KEY,
    QTSTM_POLL_KEY_RDY,
    QTSTM_POLL_ALLKEY,
    QTSTM_POLL_ALLKEY_RDY,
    QTSTM_GET_STATUS,
    QTSTM_GET_STATUS_RDY,
    QTSTM_GET_FMEA,
    QTSTM_GET_FMEA_RDY,
    QTSTM_GET_ERROR_ALLKEY,
    QTSTM_GET_ERROR_ALLKEY_RDY,
    QTSTM_DOWN,
    QTSTM_CLR_ERROR,
} t_qt_statemachine;

typedef enum
{
    QTW_DO_STUFF,
    QTW_DO_CMD,
    QTW_DO_SPI,
} t_work_state;

typedef enum
{
    QTS_PHASE0,
    QTS_PHASE1,
    QTS_PHASE2,
    QTS_PHASE3,
    QTS_END,
} t_spi_phase;

typedef enum
{
    QTS_NTHR,
    QTS_NDRIFT,
    QTS_NDIL,
    QTS_FDIL,
    QTS_NRD,
    QTS_BL,
    QTS_AKS,
    QTS_SSYNC,
    QTS_MSYNC,
    QTS_BS,
    QTS_LSL
}   t_qtsetup;

typedef struct
{
    u16 byteptr;
    u8  bitptr;
    u8  bitlng;
    u16 defval;
    u16 sectcnt;
} t_qtsetup_tlb;

static const t_qtsetup_tlb qtsetup_tlb[] =
{
    {0,0,4,6,24},
    {0,4,4,10,24},

    {							 //enabled keys:0-13
	24,0,4,2,14
    },
    {24,4,4,5,14},
    {							 //disabled keys:14-15
	38,0,4,0,2
    },
    {38,4,4,0,2},
    {							 //enabled keys:16-21
	40,0,4,2,6
    },
    {40,4,4,5,6},
    {							 //disabled key:22,23
	42,0,4,0,2
    },
    {42,4,4,0,2},

    {48,0,8,20,24},
    {72,4,2,2,24},
    {72,6,1,0,24},
    {72,7,1,0,24},
    {96,6,1,0,1},
    {97,0,4,1,1},
    {98,0,8,100,1},
    {99,0,8,0,1},
    {0,0,0,0,0},
};

#define QTSETUP_LNG 101

static void qt60248_input_work(struct work_struct *w);
//static DECLARE_DELAYED_WORK(report_work, qt60248_input_work);
//static struct qt60248_data * qt60248_data_ptr;

struct qt60248_data {
	struct input_dev *input;
	struct spi_device *spi;
	unsigned int irq;
	unsigned short keycodes[ARRAY_SIZE(qt60248_key2code)];
	unsigned polling_delay;
	struct workqueue_struct *workq;
	struct delayed_work report_work;
	t_qt_statemachine   qt_statemachine;
	t_work_state work_state;
	t_spi_phase spi_phase;
	unsigned spi_data_cnt;
	u8 setup_crc;
	t_qtcmd qtcmd;
	u32 qtcmd_response;
	u8 *setup;
	unsigned crc_ecnt;
	unsigned qtcmd_ecnt;
	u8 cmd;
	u32 new_keys;
	u32 last_keys;
	u8 fmea_error;
	u32 key_error;
	u8 ecnt;
	unsigned gpio;
	bool delay;
};

static int qt_spi_write(struct spi_device *spi, u8 data)
{
    return spi_write (spi,&data,sizeof(u8));
}


static int qt_spi_read(struct spi_device *spi)
{
    int ret;
    u8 datat = (u8) QT_NULL;
    u8 datar;

    struct spi_transfer t = {
	.rx_buf         = &datar,
	.tx_buf		= &datat,
	.len            = sizeof(u8),
    };
    struct spi_message  m;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    ret = spi_sync(spi, &m);

    if (ret < 0)
    {
	dev_err(&spi->dev, "spi transfer error\n");
	return ret;
    }
    else
	return (int)datar;
}

// 8 bits crc calculation. Initial value is 0.
// polynomial = X8 + X5 + X4 + 1
// data is an 8 bit number; crc is a 8 bit number
static u8 eight_bit_crc(u8 crc, u8 data)
{
    u8 index;					 // shift counter
    u8 fb;
    index = 8;					 // initialise the shift counter

    do
    {
	fb = (crc ^ data) & 0x01;
	data >>= 1;
	crc >>= 1;
	if (fb)
	{
	    crc ^= 0x8c;
	}
    } while( --index );

    return crc;
}

static void create_setup(u8 *setup,t_qtsetup_tlb *tlb)
{
    int i,j;
    u8 crc = 0;

    for (i = 0; i < 101; i++)
	setup[i]=0;

    for (i = 0; tlb[i].sectcnt; i++)
    {
	for (j = 0; j < tlb[i].sectcnt; j++)
	{
	    setup[tlb[i].byteptr + j] |= ((tlb[i].defval & ((1 << tlb[i].bitlng)-1)) << tlb[i].bitptr);
	}
    }
    for (i = 0; i < 100; i++)
	crc=eight_bit_crc(crc, setup[i]);
    setup[100] = crc;
}

static int qt_command (struct qt60248_data *qtdata)
{

    struct spi_device *spi = qtdata->spi;

    u8 data;
    u8 cmd = (u8)qtdata->qtcmd;

    if (qtdata->spi_phase == QTS_PHASE0)
    {
	//printk("qt60248 spi phase0\n");
	qt_spi_write(spi, (u8) cmd);
	qtdata->spi_phase = QTS_PHASE1;
	return 0;
    }
    else if (qtdata->spi_phase == QTS_PHASE1)
    {
	//printk("qt60248 spi phase1\n");
	qtdata->spi_phase = QTS_PHASE2;
	switch (cmd)			//must repear command
	{
	    case QT_LD_SETUP:
	    case QT_CAL_ALL:
	    case QT_RESET:
	    case QT_CAL_KEY:
		qt_spi_write(spi, (u8) cmd);
		return 0;
	    case QT_REP_ALLKEY:
	    case QT_ERR_ALLKEY:
		qtdata->qtcmd_response = 0;
		qtdata->setup_crc = eight_bit_crc(0,cmd);
		qtdata->spi_data_cnt  = 0;
		break;
	    default:
		break;
	}
    }
    else if (qtdata->spi_phase == QTS_END)
    {
	qtdata->work_state = QTW_DO_STUFF;
	//printk("qt60248: end of command\n");
	return 0;
    }

    switch (cmd)
    {
	case QT_LD_SETUP:
	    switch (qtdata->spi_phase)
	    {
		case QTS_PHASE2:
		    if ((u8)qt_spi_read(spi) != 0xFE)
		    {
			qtdata->spi_phase = QTS_END;
			return -2;
		    }
		    if (qtdata->setup == NULL)
		    {
			qtdata->spi_phase = QTS_END;
			return -2;
		    }

		    qtdata->setup_crc = 0;
		    qtdata->spi_data_cnt = 0; 
		    qtdata->spi_phase = QTS_PHASE3;
		    return 0;
		case QTS_PHASE3:
		    if (qtdata->spi_data_cnt < 100)
		    {
			data = qtdata->setup[qtdata->spi_data_cnt];
			qtdata->setup_crc = eight_bit_crc(qtdata->setup_crc, data);
			qt_spi_write(spi, data);
		    }
		    else if (qtdata->spi_data_cnt == 100)
			qt_spi_write(spi, qtdata->setup_crc);
		    else if (qtdata->spi_data_cnt == 101)
		    {
			if ((u8)qt_spi_read(spi) != 0xFE)
			{
			    qtdata->spi_phase = QTS_END;
			    return -3;
			}
			qtdata->spi_phase = QTS_END;
			//qtdata->work_state = QTW_DO_STUFF;
		    }
		    qtdata->spi_data_cnt++;
		    return 0;
		default:
		    break;
	    }
	    break;
	case QT_CAL_KEY:
	case QT_CAL_ALL:
	    if ((u8)qt_spi_read(spi) != ~cmd)
	    {
		qtdata->spi_phase = QTS_END;
		return -2;
	    }
	    qtdata->spi_phase = QTS_END;
	    //qtdata->work_state = QTW_DO_STUFF;
	    break;

	case QT_RESET:
	    if ((u8)qt_spi_read(spi) != 0xFB)
	    {
		qtdata->spi_phase = QTS_END;
		return -2;
	    }
	    qtdata->spi_phase = QTS_END;
	    //qtdata->work_state = QTW_DO_STUFF;
	    break;

	case QT_GEN_STATUS:
	case QT_REP_1STKEY:
	case QT_FMEA_STAT:
	    switch (qtdata->spi_phase)
	    {
		case QTS_PHASE2:
		    qtdata->setup_crc = eight_bit_crc(0,cmd);
		    data = (u8)qt_spi_read(spi);
		    qtdata->setup_crc = eight_bit_crc(qtdata->setup_crc, data);
		    qtdata->qtcmd_response = data;
		    qtdata->spi_phase = QTS_PHASE3;
		    return 0;
		case QTS_PHASE3:
		    data = (u8)qt_spi_read(spi);
		    if (eight_bit_crc( qtdata->setup_crc, data))
		    {
			qtdata->spi_phase = QTS_END;
			return -3;
		    }
		    qtdata->spi_phase = QTS_END;
		    //qtdata->work_state = QTW_DO_STUFF;
		    break;
		default:
		    break;
	    }
	    break;

	case QT_REP_ALLKEY:
	case QT_ERR_ALLKEY:
	    if (qtdata->spi_data_cnt < 3)
	    {
		qtdata->qtcmd_response <<= 8;
		data = (u8)qt_spi_read(spi);
		qtdata->setup_crc = eight_bit_crc(qtdata->setup_crc, data);
		qtdata->qtcmd_response += data;
	    }
	    else if (qtdata->spi_data_cnt == 3)
	    {
		data = (u8)qt_spi_read(spi);
		if (eight_bit_crc( qtdata->setup_crc, data))
		{
		    qtdata->spi_phase = QTS_END;
		    return -3;
		}
		qtdata->spi_phase = QTS_END;
		//qtdata->work_state = QTW_DO_STUFF;
	    }
	    qtdata->spi_data_cnt++;
	    break;
	case QT_EEPROM_CRC:
	case QT_LAST_CMD:
	    qtdata->qtcmd_response = (u8)qt_spi_read(spi);
	    //printk("qt60248: QT_LAST_CMD: %u\n",qtdata->qtcmd_response);
	    qtdata->spi_phase = QTS_END;
	    //qtdata->work_state = QTW_DO_STUFF;
	    break;
	default:
	    qtdata->work_state = QTW_DO_STUFF;
	    return -4;
    }
    return 0;
}



static void queue_qtcommand (struct qt60248_data *qtdata, t_qtcmd cmd)
{
    qtdata->qtcmd = cmd;
    qtdata->work_state = QTW_DO_CMD;
    qtdata->spi_phase = QTS_PHASE0;
    qtdata->spi_data_cnt = 0;
    qtdata->qtcmd_ecnt = 0;
}

static int  do_qtstuff (struct qt60248_data *qtdata)
{
    u8 data;
    int key_pushed;


    switch (qtdata->qt_statemachine)
    {
	case QTSTM_CHECK_RDY:
	    //printk("qt60248: CHECK RDY\n");
	    if ((u8)qtdata->qtcmd_response == 0xF0)
	    {
		DBG(KERN_INFO "Got good check byte\n");
		qtdata->delay = false;
		qtdata->qt_statemachine = QTSTM_CHK_SETUPCRC;
		break;
	    }
	    //printk("qt60248: Bad check byte: %u\n",qtdata->qtcmd_response);
	case QTSTM_CHECK:
	    //printk("qt60248: CHECK\n");
	    qtdata->ecnt = 0;
	    qtdata->delay = false;
	    queue_qtcommand (qtdata, QT_LAST_CMD);
	    qtdata->qt_statemachine = QTSTM_CHECK_RDY;
	    break;
	case QTSTM_CHK_SETUPCRC:
	    printk("qt60248: CHECK SETUPCRC\n");

	    qtdata->delay = false;

	    qtdata->setup = kzalloc(sizeof(u8)*104, GFP_KERNEL);
	    create_setup(qtdata->setup, (t_qtsetup_tlb *)(qtsetup_tlb));
	    queue_qtcommand (qtdata, QT_EEPROM_CRC);
	    qtdata->qt_statemachine = QTSTM_CHK_SETUPCRC_RDY;
	    break;
	case QTSTM_CHK_SETUPCRC_RDY:
	    qtdata->delay = false;

	    if ((u8)qtdata->qtcmd_response == qtdata->setup[100])
	    {
		kfree(qtdata->setup);
		qtdata->setup = NULL;
		qtdata->crc_ecnt = 0;
		qtdata->qt_statemachine = QTSTM_POLL_KEY;
		printk("qt60248: SETUP CRC OK\n");

		break;
	    }
	    kfree(qtdata->setup);

	    qtdata->crc_ecnt++;
	    if (qtdata->crc_ecnt == 2)
	    {
		qtdata->qt_statemachine = QTSTM_DOWN;
		break;
	    }
	    queue_qtcommand (qtdata, QT_EEPROM_CRC);
	    qtdata->qt_statemachine = QTSTM_RESET;
	    break;

	case QTSTM_RESET:
	    qtdata->delay = false;
	    printk("qt60248: RESET\n");

	    queue_qtcommand (qtdata, QT_RESET);
	    qtdata->ecnt = 0;
	    qtdata->qt_statemachine = QTSTM_CHECK;
	    break;

	case QTSTM_POLL_KEY:
	    qtdata->delay = false;

	    queue_qtcommand (qtdata, QT_REP_1STKEY);
	    qtdata->qt_statemachine = QTSTM_POLL_KEY_RDY;
	    break;

	case QTSTM_POLL_KEY_RDY:
	    data = (u8)qtdata->qtcmd_response;
	    if (data & ANY_ERROR)
	    {
		qtdata->qt_statemachine = QTSTM_GET_STATUS;
		qtdata->delay = false;
		break;
	    }
	    if (data & MORE_THAN1)
	    {
		qtdata->qt_statemachine = QTSTM_POLL_ALLKEY;
		qtdata->delay = false;
		break;
	    }
	    
	    qtdata->delay = true;
	    qtdata->qt_statemachine = QTSTM_POLL_KEY;
	    if (data < 24)
	    {
		qtdata->new_keys = ((1 << (data & 0x1F)) & KEY_MASK);
		return 1;
	    }
	    else
	    {
		qtdata->new_keys = 0;
		return 1;
	    }
	    break;

	case QTSTM_POLL_ALLKEY:
	    qtdata->delay = false;

	    queue_qtcommand (qtdata, QT_REP_ALLKEY);
	    qtdata->qt_statemachine = QTSTM_POLL_ALLKEY_RDY;
	    break;

	case QTSTM_POLL_ALLKEY_RDY:

	    qtdata->delay = true;
	    qtdata->qt_statemachine = QTSTM_POLL_KEY;

	    qtdata->new_keys = qtdata->qtcmd_response  & KEY_MASK;
	    return 1;

	case QTSTM_GET_STATUS:
	    qtdata->delay = false;

	    queue_qtcommand (qtdata, QT_GEN_STATUS);
	    qtdata->qt_statemachine = QTSTM_GET_STATUS_RDY;
	    break;

	case QTSTM_GET_STATUS_RDY:
	    
	    qtdata->delay = false;
	    data = (u8)qtdata->qtcmd_response;
	    if (data & COMM_ERR)
	    {
		qtdata->qt_statemachine = QTSTM_CLR_ERROR;
		break;
	    }
	    if (data & FMEA_ERR)
	    {
		qtdata->qt_statemachine = QTSTM_GET_FMEA;
		break;
	    }
	    if (data & CAL_ERR)
	    {
		qtdata->qt_statemachine = QTSTM_GET_ERROR_ALLKEY;
		break;
	    }
	    qtdata->qt_statemachine = QTSTM_CLR_ERROR;
	    break;

	case QTSTM_GET_FMEA:
	    qtdata->delay = false;

	    queue_qtcommand (qtdata, QT_FMEA_STAT);
	    qtdata->qt_statemachine = QTSTM_GET_FMEA_RDY;
	    break;

	case QTSTM_GET_FMEA_RDY:
	    qtdata->delay = false;
	    qtdata->fmea_error = (u8)qtdata->qtcmd_response;
	    qtdata->qt_statemachine = QTSTM_DOWN;
	    break;

	case QTSTM_GET_ERROR_ALLKEY:
	    qtdata->delay = false;

	    queue_qtcommand (qtdata, QT_ERR_ALLKEY);
	    qtdata->qt_statemachine = QTSTM_GET_ERROR_ALLKEY_RDY;
	    break;
	case QTSTM_GET_ERROR_ALLKEY_RDY:
	    qtdata->delay = false;
	    qtdata->key_error = qtdata->qtcmd_response;
	    qtdata->qt_statemachine = QTSTM_RESET;
	    break;

	case QTSTM_DOWN:
	    dev_err(&qtdata->spi->dev, "FATAL error, qt60248 is down\n");
	    return -1;

	case QTSTM_CLR_ERROR:
	    qtdata->delay = false;
	    queue_qtcommand (qtdata, QT_LAST_CMD);
	    qtdata->qt_statemachine = QTSTM_POLL_KEY;
	    break;

    }
    return 0;
}


static irqreturn_t qt60248_interrupt(int irq, void *dev_id)
{
	struct qt60248_data *qtdata = dev_id;
	queue_work(qtdata->workq, &qtdata->report_work.work);
	return IRQ_HANDLED;
}


static void qt60248_input_work(struct work_struct *work)
{
	struct qt60248_data *qtdata = container_of( work, struct qt60248_data, report_work.work );
	int ret = 0;

	if (qtdata->work_state == QTW_DO_STUFF)
	{
	    ret = do_qtstuff (qtdata);
	    if (ret > 0)
	    {

		int i;
		u32 mask=1;
		u32 new_keys = qtdata->new_keys;
		u32 keyval;

		for (i = 0; i < ARRAY_SIZE(qt60248_key2code); i++) {
			keyval = new_keys & mask;
			if ((qtdata->last_keys & mask) != keyval)
				input_report_key(qtdata->input, qtdata->keycodes[i], keyval);
			mask <<= 1;
		}
		input_sync(qtdata->input);

		qtdata->last_keys = new_keys;
	    }
	}

	if(qtdata->work_state == QTW_DO_CMD)
	{
	    if (qt_command (qtdata) < 0)
	    {
		qtdata->qtcmd_ecnt++;
		if (qtdata->qtcmd_ecnt >= 3 )
		{
		    dev_err(&qtdata->spi->dev, "qt60248 RESET\n");
		    qtdata->qt_statemachine = QTSTM_RESET;
		}
	    }
	}
	
	if (qtdata->work_state == QTW_DO_STUFF)
	{
	    if (qtdata->delay)
	    {
		//printk("qt60248: next stuff with delayed\n");
		queue_delayed_work(qtdata->workq, &qtdata->report_work, qtdata->polling_delay);
//		queue_delayed_work(qtdata->workq, &qtdata->report_work, HZ);
	    }
	    else
	    {
		//printk("qt60248: next stuff\n");
		queue_work(qtdata->workq, &qtdata->report_work.work);
	    }
	}
}


static int __devinit qt60248_probe(struct spi_device *spi)
{
	struct qt60248_data *data;
	struct input_dev *input;
	struct qt60248_platform_data	*pdata = spi->dev.platform_data;
	int i;
	int err;

	if (!pdata)
	{
		dev_err(&spi->dev, "Platform data absent\n");
		return -EINVAL;
	}


	if (!spi->irq) {
		dev_err(&spi->dev, "please assign the irq to this device\n");
		return -EINVAL;
	}

	data = kzalloc(sizeof(struct qt60248_data), GFP_KERNEL);
	input = input_allocate_device();
	if (!data || !input) {
		dev_err(&spi->dev, "insufficient memory\n");
		err = -ENOMEM;
		goto err_free_mem;
	}

	spi->bits_per_word = 8;
	spi_setup(spi);

	data->spi = spi;
	data->input = input;
	data->irq = spi->irq;
	data->qt_statemachine = QTSTM_CHECK;
	data->work_state = QTW_DO_STUFF;
	data->crc_ecnt = 0;
	data->ecnt = 0;

	err = gpio_request(pdata->gpio, "QT60248_RDY");
	if (err < 0)
	{
	    dev_err(&spi->dev, "fail to request rdy gpio\n");
	    goto err_free_data;
	}
	err = gpio_direction_input(pdata->gpio);
	if (err < 0)
	{
	    dev_err(&spi->dev, "fail to set direction rdy gpio\n");
	    goto err_free_data;
	}
	data->gpio = pdata->gpio;
	input->name = "QT60248 QTouch Sensor";
	input->dev.parent = &spi->dev;
	input->id.bustype = BUS_SPI;
	input->id.vendor = 0x0060;
	input->id.product = 0x0248;
	input->id.version = 0x0100;
	/* Add the keycode */
	/*******************/
	input->keycode = data->keycodes;
	input->keycodesize = sizeof(data->keycodes[0]);
	input->keycodemax = ARRAY_SIZE(qt60248_key2code);

	__set_bit(EV_KEY, input->evbit);

	for (i = 0; i < ARRAY_SIZE(qt60248_key2code); i++) {
		data->keycodes[i] = qt60248_key2code[i];
		__set_bit(qt60248_key2code[i], input->keybit);
	}

	/* Calibrate device */
	/********************/

	err = request_irq(spi->irq, qt60248_interrupt,
		IRQF_TRIGGER_RISING, spi->dev.driver->name, data);
	if (err) {
		dev_err(&spi->dev, "fail to request irq\n");
		goto err_free_mem;
	}

	/* Register the input device */
	err = input_register_device(data->input);
	if (err) {
		dev_err(&spi->dev, "Failed to register input device\n");
		goto err_free_irq;
	}
	spi_set_drvdata(spi, data);

	data->delay = false;
	data->polling_delay = msecs_to_jiffies(WAIT_MS_AFTER_POLL);
	data->workq = create_singlethread_workqueue("qt60248_wq");
	if (!data->workq) {
		dev_err(&spi->dev,"QT60248 error: can't create workqueue\n");
		err = -ENOMEM;
		goto err_free_irq;
	}
	INIT_DELAYED_WORK( &data->report_work, qt60248_input_work );

	if ( !queue_delayed_work( data->workq, &data->report_work, data->polling_delay ) )
	{
		err = -EBUSY;
		dev_err(&spi->dev,"QT60248 error: can't start workqueue\n");
		goto err_free_irq;
	}
	printk("qt60248 GPIO probe success\n");
	return 0;


err_free_irq:
	free_irq(spi->irq, data);
err_free_mem:
	input_free_device(input);
err_free_data:
	kfree(data);
	return err;

/****************************************/
}

static int __devexit qt60248_remove(struct spi_device *spi)
{
	struct qt60248_data *data = spi_get_drvdata(spi);

	/* Release IRQ */
	free_irq(spi->irq, data);
	if ( !cancel_delayed_work_sync( &data->report_work ) )
		flush_workqueue( data->workq );
	destroy_workqueue( data->workq );
	input_unregister_device(data->input);
	gpio_free(data->gpio);
	kfree(data);

	return 0;
}

static const struct spi_device_id qt60248_id_table[] = {
	{ "qt60248",	0x0248 },
	{ },
};

MODULE_DEVICE_TABLE(spi, qt60248_id_table);

#ifdef CONFIG_OF
static const struct of_device_id qt60248_of_match[] = {
	{ .compatible = "microchip,qt60248" },
	{ },
};
MODULE_DEVICE_TABLE(of, qt60248_of_match);
#endif

static struct spi_driver qt60248_driver = {
	.driver = {
		.name = "qt60248",
		.bus = &spi_bus_type,
		.of_match_table		= of_match_ptr(qt60248_of_match),
		.owner = THIS_MODULE,
	},
	.id_table = qt60248_id_table,
	.probe   = qt60248_probe,
	.remove  = __devexit_p(qt60248_remove),
};


static int __init qt60248_init(void)
{
	return spi_register_driver(&qt60248_driver);
}
module_init(qt60248_init);

static void __exit qt60248_exit(void)
{
	spi_unregister_driver(&qt60248_driver);
}
module_exit(qt60248_exit);





MODULE_AUTHOR("Gabor Wunderlich <wunderlichg@t-online.hu>");
MODULE_DESCRIPTION("Driver for QT60248 QTouch sensor");
MODULE_LICENSE("GPL");
