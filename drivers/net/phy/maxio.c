/*
 * drivers/net/phy/maxio.c
 *
 * Driver for maxio PHYs
 *
 * Author: zhao yang <yang_zhao@maxio-tech.com>
 *
 * Copyright (c) 2004 maxio technology, Inc.
 */
#include <linux/bitops.h>
#include <linux/phy.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/netdevice.h>

#define MAXIO_PAGE_SELECT			    0x1f
#define MAXIO_MAE0621A_INER				0x12
#define MAXIO_MAE0621A_INER_LINK_STATUS	BIT(4)
#define MAXIO_MAE0621A_INSR				0x1d
#define MAXIO_MAE0621A_TX_DELAY			(BIT(6)|BIT(7))
#define MAXIO_MAE0621A_RX_DELAY			(BIT(4)|BIT(5))
#define MAXIO_MAE0621A_CLK_MODE_REG      0x02
#define MAXIO_MAE0621A_WORK_STATUS_REG   0x1d


int maxio_read_paged(struct phy_device *phydev, int page, u32 regnum)
{
	int ret = 0, oldpage;

	oldpage = phy_read(phydev, MAXIO_PAGE_SELECT);
	if (oldpage >= 0) {
        phy_write(phydev, MAXIO_PAGE_SELECT, page);
	    ret = phy_read(phydev, regnum);
    }
	phy_write(phydev, MAXIO_PAGE_SELECT, oldpage);

	return ret;
}

int maxio_write_paged(struct phy_device *phydev, int page, u32 regnum, u16 val)
{
	int ret = 0, oldpage;

	oldpage = phy_read(phydev, MAXIO_PAGE_SELECT);
	if (oldpage >= 0) {
        phy_write(phydev, MAXIO_PAGE_SELECT, page);
		ret = phy_write(phydev, regnum, val);
    }
	phy_write(phydev, MAXIO_PAGE_SELECT, oldpage);

	return ret;
}

static int maxio_mae0621a_clk_init(struct phy_device *phydev)
{    
	u32 workmode,clkmode,oldpage;
    
	oldpage = phy_read(phydev, MAXIO_PAGE_SELECT);
	if (oldpage == 0xFFFF)	{
		oldpage = phy_read(phydev, MAXIO_PAGE_SELECT);
	}

    //soft reset
    phy_write(phydev, MAXIO_PAGE_SELECT, 0x0);
    phy_write(phydev, MII_BMCR, BMCR_RESET | phy_read(phydev, MII_BMCR));

    //get workmode
    phy_write(phydev, MAXIO_PAGE_SELECT, 0xa43);
    workmode = phy_read(phydev, MAXIO_MAE0621A_WORK_STATUS_REG);

    //get clkmode
    phy_write( phydev,  MAXIO_PAGE_SELECT, 0xd92 );
    clkmode = phy_read( phydev, MAXIO_MAE0621A_CLK_MODE_REG );
        
    //abnormal
    if (0 == (workmode&BIT(5))) {
        if (0 == (clkmode&BIT(8))) {
            //oscillator
            phy_write(phydev, 0x02, clkmode | BIT(8));
            printk("****maxio_mae0621a_clk_init**clkmode**0x210a: 0x%x\n", phydev->phy_id);
        } else {
            //crystal
            printk("****maxio_mae0621a_clk_init**clkmode**0x200a: 0x%x\n", phydev->phy_id);
            phy_write(phydev, 0x02, clkmode &(~ BIT(8)));
        }
    }
    phy_write(phydev, MAXIO_PAGE_SELECT, 0x0);

	phy_write(phydev, MAXIO_PAGE_SELECT, oldpage);

    return 0;
}

static int maxio_read_mmd(struct phy_device *phydev, int devnum, u16 regnum)
{
	int ret = 0, oldpage;
	oldpage = phy_read(phydev, MAXIO_PAGE_SELECT);

	if (devnum == MDIO_MMD_AN && regnum == MDIO_AN_EEE_ADV) {// eee info
		phy_write(phydev, MAXIO_PAGE_SELECT ,0);
		phy_write(phydev, 0xd, MDIO_MMD_AN);
		phy_write(phydev, 0xe, MDIO_AN_EEE_ADV);
		phy_write(phydev, 0xd, 0x4000 | MDIO_MMD_AN);
		ret = phy_read(phydev, 0x0e);
	} else {
		ret = -EOPNOTSUPP;
	}
	phy_write(phydev, MAXIO_PAGE_SELECT, oldpage);

	return ret;
}

static int maxio_write_mmd(struct phy_device *phydev, int devnum, u16 regnum, u16 val)
{
	int ret = 0, oldpage;
	oldpage = phy_read(phydev, MAXIO_PAGE_SELECT);

	if (devnum == MDIO_MMD_AN && regnum == MDIO_AN_EEE_ADV) { // eee info       
		phy_write(phydev, MAXIO_PAGE_SELECT ,0);
		ret |= phy_write(phydev, 0xd, MDIO_MMD_AN);
		ret |= phy_write(phydev, 0xe, MDIO_AN_EEE_ADV);
		ret |= phy_write(phydev, 0xd, 0x4000 | MDIO_MMD_AN);
		ret |= phy_write(phydev, 0xe, val);
        msleep(100);
        ret |= genphy_restart_aneg(phydev);
        
	} else {
		ret = -EOPNOTSUPP;
	}
	phy_write(phydev, MAXIO_PAGE_SELECT, oldpage);

	return ret;
}

static int maxio_mae0621a_config_aneg(struct phy_device *phydev)
{
	return genphy_config_aneg(phydev);
}


static int maxio_mae0621a_config_init(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	u16 val;
	int ret;
	u32 broken = 0;
	
	maxio_mae0621a_clk_init(phydev);

	//disable eee
	printk("eee value: 0x%x \n",maxio_read_mmd(phydev, MDIO_MMD_AN, MDIO_AN_EEE_ADV));    
	maxio_write_mmd(phydev, MDIO_MMD_AN, MDIO_AN_EEE_ADV, 0);
	printk("eee value: 0x%x \n",maxio_read_mmd(phydev, MDIO_MMD_AN, MDIO_AN_EEE_ADV));    
	broken |= MDIO_EEE_100TX;
	broken |= MDIO_EEE_1000T;
	phydev->eee_broken_modes = broken;

	/* Set green LED for Link, yellow LED for Active */
	phy_write(phydev, MAXIO_PAGE_SELECT, 0xd04);
	phy_write(phydev, 0x10, 0x617f);
	phy_write(phydev, MAXIO_PAGE_SELECT, 0x0);
 	
	//adjust TX/RX delay
	switch (phydev->interface) {
	case PHY_INTERFACE_MODE_RGMII:
		val = 0;
		break;
	case PHY_INTERFACE_MODE_RGMII_ID:
		val = MAXIO_MAE0621A_TX_DELAY | MAXIO_MAE0621A_RX_DELAY;
		break;
	case PHY_INTERFACE_MODE_RGMII_RXID:
		val = MAXIO_MAE0621A_RX_DELAY;
		break;
	case PHY_INTERFACE_MODE_RGMII_TXID:
		val = MAXIO_MAE0621A_TX_DELAY;
		break;
	default: /* the rest of the modes imply leaving delays as is. */
		return 0;
	}

    ret = maxio_read_paged(phydev, 0xd96, 0x0);
	if (ret < 0) {
		dev_err(dev, "Failed to update the TX delay register\n");
		return ret;
	} 

    ret =maxio_write_paged(phydev, 0xd96, 0x0, val|ret );
	if (ret < 0) {
		dev_err(dev, "Failed to update the TX delay register\n");
		return ret;
	} else if (ret == 0) {
		dev_dbg(dev,
			"2ns  delay was already %s (by pin-strapping RXD1 or bootloader configuration)\n",
			val ? "enabled" : "disabled");
	} 

	return 0;
}


static int maxio_mae0621a_resume(struct phy_device *phydev)
{
	int ret = genphy_resume(phydev);

	ret |= phy_write(phydev, MII_BMCR, BMCR_RESET | phy_read(phydev, MII_BMCR));

	msleep(20);

	return ret;
}
int maxio_mae0621a_suspend(struct phy_device *phydev)
{
    genphy_suspend(phydev);
    phy_write(phydev, MAXIO_PAGE_SELECT ,0);

	return 0;
}


static int maxio_mae0621a_status(struct phy_device *phydev)
{
	return genphy_read_status(phydev);
}
static int maxio_mae0621a_probe(struct phy_device *phydev)
{
	return maxio_mae0621a_clk_init(phydev);
}

static struct phy_driver maxio_nc_drvs[] = {
    {
        .phy_id		= 0x7b744411,
		.phy_id_mask	= 0x7fffffff,
        .name       = "MAE0621A Gigabit Ethernet",
		.features	= PHY_GBIT_FEATURES,		
        .probe          = maxio_mae0621a_probe,
		.config_init	= maxio_mae0621a_config_init,
        .config_aneg    = maxio_mae0621a_config_aneg,
        .read_status    = maxio_mae0621a_status,
        .suspend    = maxio_mae0621a_suspend,
        .resume     = maxio_mae0621a_resume,
     },
};

module_phy_driver(maxio_nc_drvs);



MODULE_DESCRIPTION("Maxio PHY driver");
MODULE_AUTHOR("Zhao Yang");
MODULE_LICENSE("GPL");

