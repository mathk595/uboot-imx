/*
 * Copyright 2019 Keith & Koep GmbH
 */

#include <asm/arch/sys_proto.h>
#include "kuk_boards.h"


#define FUSE14_MODULE(x)        ((x & 0xF0000000)>>28)
#define FUSE14_MODULE_TRIZEPS   0x1
#define FUSE14_MODULE_MYON      0x2
#define FUSE14_MODULE_SBCSOM    0x4
#define FUSE14_MODULE_TRIZEPSr  (FUSE14_MODULE_MYON|FUSE14_MODULE_SBCSOM)     // 6 Fix if accidently burned to be Myon or SBCSOM
#define FUSE14_MODULE_MYONr     (FUSE14_MODULE_TRIZEPS|FUSE14_MODULE_SBCSOM)  // 5 Fix if accidently burned to be Trizeps or SBCSOM
#define FUSE14_MODULE_SBCSOMr   (FUSE14_MODULE_TRIZEPS|FUSE14_MODULE_MYON)    // 3 Fix if accidently burned to be Myon or Trizeps
#define FUSE14_USE_OTHER_FUSE   0xF  // Fuse Invalid, Use other Fuse.

#define FUSE14_RAM(x)           ((x & 0x0F000000)>>24)   
#define FUSE14_RAM_UNDEF        0x0
#define FUSE14_RAM_1GB          0x1
#define FUSE14_RAM_2GB          0x2 // Mono-Die
#define FUSE14_RAM_4GB          0x4 // Dual-Die
#define FUSE14_RAM_8GB          0x8 // Dual-Die
#define FUSE14_RAM_2GB_DUALDIE  0xA // reserve/not use for now: 2GB Dual-Die; Engineering Samples without burned fuse used it.
#define FUSE14_RAM_512MB        0xE
#define FUSE14_RAM_AUTODETECT   0xF

#define FUSE14_PCBREV(x)        ((x & 0x00F00000)>>20)   

#define FUSE14_TEMP(x)          ((x & 0x000C0000)>>18)
#define FUSE14_TEMP_UNDEF       0x0
#define FUSE14_TEMP_COMMERCIAL  0x1
#define FUSE14_TEMP_EXTENDED    0x2
#define FUSE14_TEMP_INDUSTRIAL  0x3

#define FUSE14_EXTRA(x)         ((x & 0x00030000)>>16)
#define FUSE14_EXTRA_UNDEF      0x0
#define FUSE14_EXTRA_1V8        0x1
#define FUSE14_EXTRA_3V3        0x2
#define FUSE14_EXTRA_CUSTOM     0x3

int kuk_GetOTP( int key, int defaultval)
{
    int ret = defaultval;

/*
    |....|                  Module  (0xF.......=next , 0x1.......=Trizeps,0x2.......=Myon)
    |    |....|             RAM     (0x.F......)
    |    |    |....|        PCB-Rev (0x..F.....)            
    |    |    |    |..  |   Temp    (0x...0....=undef, 0x...4....= Con  , 0x...8....= Ext   , 0x...C....= Industrial)
    |    |    |    |  ..|   Extra   (0x...0....=undef, 0x...1....= 1V8  , 0x...2....= 3V3   , 0x...3....= Custom)

    i.e. 
    Trizeps VIII Mini V1R1 with 2GB RAM:                        fuse prog 14 0 0x1A000000
    Trizeps VIII Mini V1R2 with 2GB RAM (Dual-Die K4F6E304HB):  fuse prog 14 0 0x1A100000   // Engineering sample, typ. no fuse set
    Trizeps VIII Mini V1R1 with 1GB RAM (K4F8E304HB,K4F8E3S4HB):fuse prog 14 0 0x11100000   // Dual&Mono-Die both use 1 chip-select.
    Trizeps VIII Mini V1R2 with 2GB RAM (Mono-Die K4F6E3S4HM):  fuse prog 14 0 0x12100000        
    Trizeps VIII Mini V1R2 with 4GB RAM (Dual-Die K4FBE3D4HM):  fuse prog 14 0 0x14100000        
    Myon II V1R1 with 1GB RAM (K4F8E304HB,K4F8E3S4HB):          fuse prog 14 0 0x21000000
    Myon II V1R1 with 2GB RAM (Dual-Die K4F6E304HB):            fuse prog 14 0 0x2A000000   // Engineering sample, typ. no fuse set
    Myon II V1R1 with 2GB RAM (Mono-Die K4F6E3S4HM):            fuse prog 14 0 0x22000000
    Myon II V1R1 with 4GB RAM (Dual-Die K4FBE3D4HM):            fuse prog 14 0 0x24000000
    SBCSOM V1R1 with 1GB RAM (K4F8E304HB,K4F8E3S4HB):           fuse prog 14 0 0x41000000        
    SBCSOM V1R1 with 2GB RAM (Mono-Die K4F6E3S4HM):             fuse prog 14 0 0x42000000        
    SBCSOM V1R1 with 4GB RAM (Dual-Die K4FBE3D4HM):             fuse prog 14 0 0x44000000        

    // For now do not fuse other options:
    .. LVDS    
    .. Ethernet 
    .. WLAN     
    .. Audio 
    .. MCU
    .... FPGA 
*/    
    struct ocotp_regs *ocotp = (struct ocotp_regs *)OCOTP_BASE_ADDR;
    struct fuse_bank *bank1 = &ocotp->bank[1];
	struct fuse_bank1_regs *fuse1 =
		(struct fuse_bank1_regs *)bank1->fuse_regs;
    struct fuse_bank *bank14 = &ocotp->bank[14];  // GP10
    u32 fuse14;
    bool fuse14valid;

    fuse14 = bank14->fuse_regs[0];
    if ( FUSE14_MODULE(fuse14) == FUSE14_USE_OTHER_FUSE)
        fuse14 = bank14->fuse_regs[3];
    switch( FUSE14_MODULE(fuse14))
    {
        case FUSE14_MODULE_TRIZEPS:
        case FUSE14_MODULE_TRIZEPSr:
        case FUSE14_MODULE_MYON:
        case FUSE14_MODULE_MYONr:
        case FUSE14_MODULE_SBCSOM:
        case FUSE14_MODULE_SBCSOMr:
            fuse14valid = true;
            break;
        default:
            fuse14valid = false;
            break;
    }


    if ( key == KUK_OTP_BOOTSTORAGE)
    {
        switch( fuse1->cfg0 & 0x7000)
        {
            case 0x1000:    // SD-Card Boot
                ret = KUK_BOOTSTORAGE_SDCARD;
                break;
            case 0x2000:    // eMMC Boot
                ret = KUK_BOOTSTORAGE_EMMCxGB;
                if (( defaultval > KUK_BOOTSTORAGE_EMMCxGB)&&( defaultval <= KUK_BOOTSTORAGE_EMMC32GB))
                {   // Use assumption-value --- do not attempt to figure out size here!
                    ret = defaultval;
                }
                break;
            default:
                break;
        }
    }


    if ( key == KUK_OTP_MODULE)
    {
        // Fuse only if Myon or Trizeps
        // Which Myon/Trizeps is determined by processor
        if ( fuse14valid)
        {    
            if ((FUSE14_MODULE(fuse14)==FUSE14_MODULE_TRIZEPS)||(FUSE14_MODULE(fuse14)==FUSE14_MODULE_TRIZEPSr))
            {   // Trizeps
                if ( is_imx8mq())
                {
                    ret = KUK_MODULE_TRIZEPS8;
                }else
                if ( is_imx8mm())
                {
                    ret = KUK_MODULE_TRIZEPS8MINI;
                }else
                {   // if ( isimx8mn()) // as soon as this function is available
                    ret = KUK_MODULE_TRIZEPS8NANO;
                }
            }else
            if ((FUSE14_MODULE(fuse14)==FUSE14_MODULE_SBCSOM)||(FUSE14_MODULE(fuse14)==FUSE14_MODULE_SBCSOMr))
            {   // SBCSOM
                if ( is_imx8mm())
                {
                    ret = KUK_MODULE_SBCSOM8MINI;
                }else
                {   // if ( isimx8mn()) // as soon as this function is available
                    ret = KUK_MODULE_SBCSOM8NANO;
                }
            }else            
            {   // Myon 
                if ( is_imx8mm())
                {
                    ret = KUK_MODULE_MYON2;
                }else
                {   // if ( isimx8mn()) // as soon as this function is available
                    ret = KUK_MODULE_MYON2NANO;
                }
            }
        }
    }

    if ( key == KUK_OTP_RAMWIDTH)
    {
        // For now, not fused in OTP
    }

    if ( key == KUK_OTP_RAMSIZE)
    {   // Select between 512MB, 1G, 2G, 3G, 4G and 8G
        if ( fuse14valid)
        {
            switch( FUSE14_RAM(fuse14))
            {
                case FUSE14_RAM_512MB:
                    ret = KUK_RAMSIZE_512MB;
                    break;
                case FUSE14_RAM_1GB:
                    ret = KUK_RAMSIZE_1GB;
                    break;
                case FUSE14_RAM_2GB:
                case FUSE14_RAM_2GB_DUALDIE:
                    ret = KUK_RAMSIZE_2GB;
                    break;
                case FUSE14_RAM_4GB:
                    ret = KUK_RAMSIZE_4GB;
                    break;
                case FUSE14_RAM_8GB:
                    ret = KUK_RAMSIZE_8GB;
                    break;
                case FUSE14_RAM_AUTODETECT:
                    ret = KUK_RAMSIZE_AUTODETECT;
                    break;
                default:
                    break;
            }
        }
    }
    if ( key == KUK_OTP_RAMSKEW)
    {
        if ( fuse14valid)
        {
            switch( FUSE14_RAM(fuse14))
            {
                case FUSE14_RAM_2GB:
                    ret = 1;
                    break;
                case FUSE14_RAM_2GB_DUALDIE:
                    ret = 0;
                    break;
                default:
                    break;
            }
        }
    }

    if ( key == KUK_OTP_PCBREV)
    {   // V1R1 .. VnRn
        if ( fuse14valid)
        {
            ret = FUSE14_PCBREV( fuse14);
        }
    }

    if ( key == KUK_OTP_TEMPERATURERANGE)
    {   // Range: Consumer, Extended and Industrial
        // Usually not fused(?). Only needed for Industrial
        if ( fuse14valid)
        {
            switch( FUSE14_TEMP(fuse14))
            {
                case FUSE14_TEMP_COMMERCIAL:
                    ret = KUK_TEMP_COMMERCIAL_0_70;
                    break;
                case FUSE14_TEMP_EXTENDED:
                    ret = KUK_TEMP_EXTENDED_m25_85;
                    break;
                case FUSE14_TEMP_INDUSTRIAL:
                    ret = KUK_TEMP_INDUSTRIAL_m40_85;
                    break;
                default:
                    break;
            }
        }
    }

    if ( key == KUK_OTP_IOVOLTAGE)
    {
        if ( fuse14valid)
        {
            if ((FUSE14_MODULE(fuse14)==FUSE14_MODULE_TRIZEPS)||(FUSE14_MODULE(fuse14)==FUSE14_MODULE_TRIZEPSr))
            {   // Trizeps
                // Fuse currently has no function
            }else
            {   // Myon or SBCSOM
                switch( FUSE14_EXTRA( fuse14))
                {
                    case FUSE14_EXTRA_3V3:
                        ret = KUK_IOVOLTAGE_3V3;
                        break;
                    case FUSE14_EXTRA_CUSTOM:
                        ret = KUK_IOVOLTAGE_CUSTOM;
                        break;
                    case FUSE14_EXTRA_1V8:
                    default:
                        ret = KUK_IOVOLTAGE_1V8;
                        break;                    
                }
            }
        }
    }


    if ( key == KUK_OTP_ETHERNET)
    {   // Fuse or Autodetect (mdio)? 1

    }

    if ( key == KUK_OTP_WIRELESS)
    {   // Fuse or Autodetect (pcie)? 3

    }

    if ( key == KUK_OTP_AUDIO)
    {   // Fuse or Autodetect (i2c)? 1/2

    }

    if ( key == KUK_OTP_MCU)
    {   // Fuse or Autodetect (i2c)? 2

    }

    if ( key == KUK_OTP_LVDS)
    {   // Fuse or Autodetect (i2c)? 2

    }

    if ( key == KUK_OTP_FPGA)
    {   // Fuse or Autodetect (i2c)? 6

    }


    return ret;
}

int kuk_GetModule( void)
{
    int module = ASSUME__KUK_MODULE_TYPE;

    if ( is_imx8mq())
    {
        module = KUK_MODULE_TRIZEPS8;
    }

    module = kuk_GetOTP( KUK_OTP_MODULE, module);

#ifdef FORCE__KUK_MODULE_TYPE
    module = FORCE__KUK_MODULE_TYPE;
#endif
    return module;
}

int kuk_GetRAMWidth( void)
{
    int width = ASSUME__KUK_RAMWIDTH;

    switch ( kuk_GetModule())
    {
        case KUK_MODULE_TRIZEPS8:
        case KUK_MODULE_TRIZEPS8MINI:
        case KUK_MODULE_SBCSOM8MINI:
        case KUK_MODULE_MYON2:        
            width = KUK_RAMWIDTH_32BIT;
            break;
        case KUK_MODULE_TRIZEPS8NANO:
        case KUK_MODULE_SBCSOM8NANO:
        case KUK_MODULE_MYON2NANO:    
            width = KUK_RAMWIDTH_16BIT;
            break;
        default:
            break;
    }

    width = kuk_GetOTP( KUK_OTP_RAMWIDTH, width);

#ifdef FORCE__KUK_RAMWIDTH
    width = FORCE__KUK_RAMWIDTH;
#endif
    return width;
}

int kuk_GetRAMSize( void)
{
    int size = ASSUME__KUK_RAMSIZE;

    size = kuk_GetOTP( KUK_OTP_RAMSIZE, size);

#ifdef FORCE__KUK_RAMSIZE
    size = FORCE__KUK_RAMSIZE;
#endif
    return size;
}

int kuk_GetRAMSkew( void)
{ 
    int skew = ASSUME__KUK_RAMSKEW;

    skew = kuk_GetOTP( KUK_OTP_RAMSKEW, skew);

#ifdef FORCE__KUK_RAMSKEW
    skew = FORCE__KUK_RAMSKEW;
#endif
    return skew;
}


int kuk_GetPCBrevision( void)
{
    int rev = ASSUME__KUK_PCBREV;

    rev = kuk_GetOTP( KUK_OTP_PCBREV, rev);

#ifdef FORCE__KUK_PCBREV
    rev = FORCE__KUK_PCBREV;
#endif    
    return rev;
}

int kuk_GetTemperatureRange( void)
{
    int temp = ASSUME__KUK_TEMPERATURERANGE;

    temp = kuk_GetOTP( KUK_OTP_TEMPERATURERANGE, temp);

#ifdef FORCE__KUK_TEMPERATURERANGE
    temp = FORCE__KUK_TEMPERATURERANGE;
#endif
    return temp;
}

int kuk_GetBootStorage( void)
{
    int store = ASSUME__KUK_BOOTSTORAGE;

    switch ( kuk_GetModule())
    {
        case KUK_MODULE_TRIZEPS8:
#if (ASSUME__KUK_BOOTSTORAGE == KUK_BOOTSTORAGE_UNKNOWN)
            store = KUK_BOOTSTORAGE_EMMCxGB;
#endif            
            break;
        case KUK_MODULE_TRIZEPS8MINI:
        case KUK_MODULE_TRIZEPS8NANO:
            break;
        case KUK_MODULE_MYON2:
        case KUK_MODULE_MYON2NANO:
        case KUK_MODULE_SBCSOM8MINI:
        case KUK_MODULE_SBCSOM8NANO:
#if (ASSUME__KUK_BOOTSTORAGE == KUK_BOOTSTORAGE_UNKNOWN)
            store = KUK_BOOTSTORAGE_EMMCxGB;
#endif            
            break;
        default:
            break;
    }

    store = kuk_GetOTP( KUK_OTP_BOOTSTORAGE, store);
#ifdef FORCE__KUK_BOOTSTORAGE
    store = FORCE__KUK_BOOTSTORAGE;
#endif
    return store;
}

int kuk_GetPeripheral( int peripheral)
{
    int ret = KUK_PERIPHERAL_UNKNOWN;
    int module = kuk_GetModule();

    switch( peripheral)
    {
        case KUK_PERIPHERAL_ETHERNET:   ret = kuk_GetOTP( KUK_OTP_ETHERNET, ASSUME__PERIPHERAL_ETHERNET);   break;
        case KUK_PERIPHERAL_WIRELESS:   ret = kuk_GetOTP( KUK_OTP_WIRELESS, ASSUME__PERIPHERAL_WIRELESS);   break;
        case KUK_PERIPHERAL_AUDIO:      ret = kuk_GetOTP( KUK_OTP_AUDIO,    ASSUME__PERIPHERAL_AUDIO);      break;
        case KUK_PERIPHERAL_MCU:        ret = kuk_GetOTP( KUK_OTP_MCU,      ASSUME__PERIPHERAL_MCU);        break;
        case KUK_PERIPHERAL_LVDS:       ret = kuk_GetOTP( KUK_OTP_LVDS,     ASSUME__PERIPHERAL_LVDS);       break;
        case KUK_PERIPHERAL_FPGA:       ret = kuk_GetOTP( KUK_OTP_FPGA,     ASSUME__PERIPHERAL_FPGA);       break;
        case KUK_PERIPHERAL_IOVOLTAGE:  ret = kuk_GetOTP( KUK_OTP_IOVOLTAGE,ASSUME__PERIPHERAL_IOVOLTAGE);  break;
        default: break;
    }

    if (( module == KUK_MODULE_MYON2)||( module == KUK_MODULE_MYON2NANO ))
    {
        switch( peripheral)
        {
            case KUK_PERIPHERAL_ETHERNET:   ret = KUK_ETHERNET_NONE;   break;
            case KUK_PERIPHERAL_WIRELESS:   ret = KUK_WIRELESS_NONE;   break;
            case KUK_PERIPHERAL_MCU:        ret = KUK_MCU_NONE;        break;
            case KUK_PERIPHERAL_FPGA:       ret = KUK_FPGA_NONE;       break;
            default: break;
        }
    }else
    {
        switch( peripheral)
        {
            case KUK_PERIPHERAL_IOVOLTAGE:  ret = KUK_IOVOLTAGE_3V3;    break;
            default: break;
        }
    }

    switch( peripheral)
    {
#ifdef FORCE__PERIPHERAL_ETHERNET        
        case KUK_PERIPHERAL_ETHERNET:   ret = FORCE__PERIPHERAL_ETHERNET;   break;
#endif        
#ifdef FORCE__PERIPHERAL_WIRLESS
        case KUK_PERIPHERAL_WIRELESS:   ret = FORCE__PERIPHERAL_WIRELESS;   break;
#endif        
#ifdef FORCE__PERIPHERAL_AUDIO
        case KUK_PERIPHERAL_AUDIO:      ret = FORCE__PERIPHERAL_AUDIO;      break;
#endif        
#ifdef FORCE__PERIPHERAL_MCU
        case KUK_PERIPHERAL_MCU:        ret = FORCE__PERIPHERAL_MCU;        break;
#endif        
#ifdef FORCE__PERIPHERAL_LVDS
        case KUK_PERIPHERAL_LVDS:       ret = FORCE__PERIPHERAL_LVDS;       break;
#endif        
#ifdef FORCE__PERIPHERAL_FPGA
        case KUK_PERIPHERAL_FPGA:       ret = FORCE__PERIPHERAL_FPGA;       break;
#endif        
#ifdef FORCE__PERIPHERAL_IOVOLTAGE
        case KUK_PERIPHERAL_IOVOLTAGE:  ret = FORCE__PERIPHERAL_IOVOLTAGE;  break;
#endif        
        default: break;
    }

    return ret;
}


/*
  59 A21 . C2002 . H00 S00      Trizeps VIII Mini
  ff aff . xdddd . Hxx
  ++ Trizeps VIII mini (module)
  
     + Processor (able to autodetect)
      + RAM-Size (ramsize)
       + PCB Revision (pcbrev)
        
           + Temperature Range (temprange)
            + FPGA
             + Peripherals
              + Ethernet/WLAN
               + EMMC config (able to autodetect)

  60 A20 . C0000 . H00 S00      Myon II
  ff aff . xxddd . Hxx
  ++ Myon II (module)

     + Processor (able to autodetect)
      + RAM-Size (ramsize)
       + PCB Revision (pcbrev)

           + Temperature Range (temprange)
            + I/O voltage
             + Peripherals
              + LVDS
               + EMMC Config (able to autodetect)


f = detect through fuse
a = autodetect
x = fuse reserved but only burned if needed
d = possibly autodetect 
*/                   

// kuk_GetArticleNo()
// tries to detect the current product-number
int kuk_GetArticleNo( char *pArticle, int maxsize )
{
    int ret = 0;
    int module;
    int ramsize;
    int pcbrev;
    unsigned int speed; 
    unsigned int cpu; 

    module  = kuk_GetModule();
    ramsize = kuk_GetRAMSize(); 
    pcbrev  = kuk_GetPCBrevision();
    speed   = get_cpu_speed_grade_hz();
    cpu     = get_cpu_type(); 

    if ( maxsize <  16)
        return ret;

    pArticle[0] = 0; // zero string if no module identified.

    if ( module == KUK_MODULE_TRIZEPS8)
    {
        int eth, aud, mcu;
        pArticle[0] = '5';
        pArticle[1] = '8';
        // Get Processor Skew:
        if ( speed <= 1300000000)   
        {   // industrial
            switch( cpu)
            {
                case MXC_CPU_IMX8MD:    pArticle[2] = '0';  break;
                case MXC_CPU_IMX8MQL:   pArticle[2] = '2';  break;
                case MXC_CPU_IMX8MQ:    pArticle[2] = '3';  break;
                default:                pArticle[2] = 'x';  break;
            }
        }else
        {   // consumer
            switch( cpu)
            {
                case MXC_CPU_IMX8MD:    pArticle[2] = '5';  break;
                case MXC_CPU_IMX8MQL:   pArticle[2] = '7';  break;
                case MXC_CPU_IMX8MQ:    pArticle[2] = '8';  break;
                default:                pArticle[2] = 'x';  break;            
            }
        }

        // Get RAM-Type        
        if ( ramsize == KUK_RAMSIZE_UNKNOWN)
            pArticle[3] = 'x';
        else
        {
            switch( kuk_GetTemperatureRange())
            {
                case KUK_TEMP_EXTENDED_m25_85:      
                case KUK_TEMP_INDUSTRIAL_m40_85:
                    pArticle[3] = ramsize + '0';  
                    break;                    
                case KUK_TEMP_COMMERCIAL_0_70:     
                default:                            
                    pArticle[3] = ramsize + '5';  
                    break;
            }
        }
        if ( pcbrev == KUK_PCBREV_UNKNOWN)
            pArticle[4] = 'x';
        else
            pArticle[4] = pcbrev + '0';
        
        pArticle[5] = '.';
        
        switch ( kuk_GetPeripheral( KUK_PERIPHERAL_FPGA))
        {
            case KUK_FPGA_NONE:             pArticle[6] = '0';  break;
            case KUK_FPGA_LCMXO3LF_1300E:   pArticle[6] = '1';  break;
            case KUK_FPGA_LCMXO3LF_2100E:   pArticle[6] = '2';  break;
            case KUK_FPGA_LCMXO3LF_4300E:   pArticle[6] = '3';  break;
            case KUK_FPGA_LCMXO3L_1300E:    pArticle[6] = '4';  break;
            case KUK_FPGA_LCMXO3L_2100E:    pArticle[6] = '5';  break;
            case KUK_FPGA_LCMXO3L_4300E:    pArticle[6] = '6';  break;
            default:                        pArticle[6] = 'x';  break;
        }
        
        switch ( kuk_GetPeripheral( KUK_PERIPHERAL_LVDS))
        {
            case KUK_LVDS_NONE:             pArticle[7] = '0';  break;
            case KUK_LVDS_SN65DSI83:        pArticle[7] = '1';  break;
            case KUK_LVDS_SN65DSI85:        pArticle[7] = '2';  break;
            default:                        pArticle[7] = 'x';  break;
        }
        eth = kuk_GetPeripheral( KUK_PERIPHERAL_ETHERNET);
        aud = kuk_GetPeripheral( KUK_PERIPHERAL_AUDIO);
        mcu = kuk_GetPeripheral( KUK_PERIPHERAL_MCU);

        pArticle[8] = 'x';      // Periherals: Ethernet, Audio, Kinetis MCU
        if (( eth != KUK_ETHERNET_UNKNOWN)&&( aud == KUK_AUDIO_UNKNOWN)&&( mcu == KUK_MCU_UNKNOWN))
        {
            if ( eth == KUK_ETHERNET_NONE)
            {
                if ( aud == KUK_AUDIO_NONE)
                {
                    switch( mcu)
                    {
                        case KUK_MCU_NONE:          pArticle[8] = '0';  break;
                        case KUK_MCU_MKV10Z32VFM7:  pArticle[8] = '1';  break;
                        case KUK_MCU_MKV11Z64VFM7:  pArticle[8] = '2';  break;
                        default: break;
                    }
                }else
                {
                    switch( mcu)
                    {
                        case KUK_MCU_MKV11Z64VFM7:  pArticle[8] = '3';  break;
                        default: break;
                    }
                }
            }else
            {
                if ( aud == KUK_AUDIO_NONE)
                {
                    switch( mcu)
                    {
                        case KUK_MCU_MKV10Z32VFM7:  pArticle[8] = '4';  break;
                        case KUK_MCU_MKV11Z64VFM7:  pArticle[8] = '5';  break;
                        default: break;
                    }
                }else
                {
                    switch( mcu)
                    {
                        case KUK_MCU_MKV11Z64VFM7:  pArticle[8] = '6';  break;
                        case KUK_MCU_NONE:          pArticle[8] = '7';  break;
                        default: break;
                    }                    
                }
            }
        }

        switch( kuk_GetPeripheral( KUK_PERIPHERAL_WIRELESS))
        {
            case KUK_WIRELESS_NONE:         pArticle[9] = '0';  break;
            case KUK_WIRELESS_LAIRD_SU60:   pArticle[9] = '1';  break;
            case KUK_WIRELESS_LAIRD_ST60:   pArticle[9] = '1';  break;
            default:                        pArticle[9] = 'x';  break;
        }        
        
        switch( kuk_GetBootStorage())
        {
            case KUK_BOOTSTORAGE_SDCARD:    pArticle[10]= '0'; break;
            case KUK_BOOTSTORAGE_EMMC4GB:   pArticle[10]= '1'; break;
            case KUK_BOOTSTORAGE_EMMC8GB:   pArticle[10]= '2'; break;
            case KUK_BOOTSTORAGE_EMMC16GB:  pArticle[10]= '3'; break;
            default:                        pArticle[10]= 'x'; break;
        }
       
        pArticle[11]= '.';     
        pArticle[12]= 'H';     
        pArticle[13]= 'x';     
        pArticle[14]= 'x';  
        pArticle[15]= 0;
        ret = 16;   
    }
    if (( module == KUK_MODULE_TRIZEPS8MINI)||( module == KUK_MODULE_TRIZEPS8NANO))
    {
        int lvds, aud, mcu, eth, wlan;
        if ( module == KUK_MODULE_TRIZEPS8MINI)
        {   // Mini
            pArticle[0] = '5';
            pArticle[1] = '9';
            // Get Processor Skew:
            if ( speed <= 1600000000)   
            {   // industrial
                switch( cpu)
                {
                    case MXC_CPU_IMX8MMSL:  pArticle[2] = '1';  break;
                    case MXC_CPU_IMX8MMS:   pArticle[2] = '3';  break;
                    case MXC_CPU_IMX8MMDL:  pArticle[2] = '5';  break;
                    case MXC_CPU_IMX8MMD:   pArticle[2] = '7';  break;
                    case MXC_CPU_IMX8MML:   pArticle[2] = '9';  break;
                    case MXC_CPU_IMX8MM:    pArticle[2] = 'B';  break;
                    default:                pArticle[2] = 'x';  break;
                }
            }else
            {   // consumer
                switch( cpu)
                {
                    case MXC_CPU_IMX8MMSL:  pArticle[2] = '0';  break;
                    case MXC_CPU_IMX8MMS:   pArticle[2] = '2';  break;
                    case MXC_CPU_IMX8MMDL:  pArticle[2] = '4';  break;
                    case MXC_CPU_IMX8MMD:   pArticle[2] = '6';  break;
                    case MXC_CPU_IMX8MML:   pArticle[2] = '8';  break;
                    case MXC_CPU_IMX8MM:    pArticle[2] = 'A';  break;
                    default:                pArticle[2] = 'x';  break;            
                }
            }
        }else
        {   // Nano:
            pArticle[0] = 'x';
            pArticle[1] = 'x';        
            // Get Processor Skew:
            pArticle[2] = 'x';
        }
        
        switch ( ramsize)
        {
            case KUK_RAMSIZE_512MB: pArticle[3] = '0';  break;
            case KUK_RAMSIZE_1GB:   pArticle[3] = '1';  break;
            case KUK_RAMSIZE_2GB:   pArticle[3] = '2';  break;
            case KUK_RAMSIZE_3GB:   pArticle[3] = '3';  break;
            case KUK_RAMSIZE_4GB:   pArticle[3] = '4';  break;
            case KUK_RAMSIZE_8GB:   pArticle[3] = '8';  break;
            default:                pArticle[3] = 'x';  break;
        }
        if ( pcbrev == KUK_PCBREV_UNKNOWN)
            pArticle[4] = 'x';
        else
            pArticle[4] = pcbrev + '0';
        
        pArticle[5] = '.';

        switch( kuk_GetTemperatureRange())
        {
            case KUK_TEMP_COMMERCIAL_0_70:      pArticle[6] = 'C';  break;
            case KUK_TEMP_EXTENDED_m25_85:      pArticle[6] = 'E';  break;
            case KUK_TEMP_INDUSTRIAL_m40_85:    pArticle[6] = 'I';  break;
            default:                            pArticle[6] = 'x';  break;
        }      

        switch ( kuk_GetPeripheral( KUK_PERIPHERAL_FPGA))
        {
            case KUK_FPGA_NONE:             pArticle[7] = '0';  break;
            case KUK_FPGA_LCMXO3LF_1300E:   pArticle[7] = '1';  break;
            case KUK_FPGA_LCMXO3LF_2100E:   pArticle[7] = '2';  break;
            case KUK_FPGA_LCMXO3LF_4300E:   pArticle[7] = '3';  break;
            case KUK_FPGA_LCMXO3L_1300E:    pArticle[7] = '4';  break;
            case KUK_FPGA_LCMXO3L_2100E:    pArticle[7] = '5';  break;
            case KUK_FPGA_LCMXO3L_4300E:    pArticle[7] = '6';  break;
            default:                        pArticle[7] = 'x';  break;
        }

        lvds = kuk_GetPeripheral( KUK_PERIPHERAL_MCU);
        aud = kuk_GetPeripheral( KUK_PERIPHERAL_AUDIO);
        mcu = kuk_GetPeripheral( KUK_PERIPHERAL_MCU);
        pArticle[8] = 'x';      // Periherals: LVDS, Audio, Kinetis MCU
        if (( lvds != KUK_LVDS_UNKNOWN)&&( aud != KUK_AUDIO_UNKNOWN)&&( mcu != KUK_MCU_UNKNOWN))
        {
            if ( lvds == KUK_LVDS_NONE)
            {
                if ( aud == KUK_AUDIO_NONE)
                {
                    switch( mcu)
                    {
                        case KUK_MCU_NONE:          pArticle[8] = '0';  break;
                        case KUK_MCU_MKV10Z32VFM7:  pArticle[8] = '1';  break;
                        case KUK_MCU_MKV11Z64VFM7:  pArticle[8] = '2';  break;
                        default: break;
                    }
                }else
                {
                    switch( mcu)
                    {
                        case KUK_MCU_NONE:          pArticle[8] = '3';  break;
                        case KUK_MCU_MKV10Z32VFM7:  pArticle[8] = '4';  break;
                        case KUK_MCU_MKV11Z64VFM7:  pArticle[8] = '5';  break;
                        default: break;
                    }
                }
            }else
            if ( lvds == KUK_LVDS_SN65DSI83)
            {
                if ( aud == KUK_AUDIO_NONE)
                {
                    switch( mcu)
                    {
                        case KUK_MCU_NONE:          pArticle[8] = '6';  break;
                        case KUK_MCU_MKV10Z32VFM7:  pArticle[8] = '7';  break;
                        case KUK_MCU_MKV11Z64VFM7:  pArticle[8] = '8';  break;
                        default: break;
                    }
                }else
                {
                    switch( mcu)
                    {
                        case KUK_MCU_NONE:          pArticle[8] = '9';  break;
                        case KUK_MCU_MKV10Z32VFM7:  pArticle[8] = 'A';  break;
                        case KUK_MCU_MKV11Z64VFM7:  pArticle[8] = 'B';  break;
                        default: break;
                    }
                }
            }else
            if ( lvds == KUK_LVDS_SN65DSI85)
            {
                if ( aud == KUK_AUDIO_NONE)
                {
                    switch( mcu)
                    {
                        case KUK_MCU_NONE:          pArticle[8] = 'C';  break;
                        case KUK_MCU_MKV10Z32VFM7:  pArticle[8] = 'D';  break;
                        case KUK_MCU_MKV11Z64VFM7:  pArticle[8] = 'E';  break;
                        default: break;
                    }
                }else
                {
                    switch( mcu)
                    {
                        case KUK_MCU_NONE:          pArticle[8] = 'F';  break;
                        case KUK_MCU_MKV10Z32VFM7:  pArticle[8] = 'G';  break;
                        case KUK_MCU_MKV11Z64VFM7:  pArticle[8] = 'H';  break;
                        default: break;
                    }
                }
            }

        }
        
        eth = kuk_GetPeripheral( KUK_PERIPHERAL_ETHERNET);
        wlan = kuk_GetPeripheral( KUK_PERIPHERAL_WIRELESS);
        pArticle[9] = 'x';      // Ethernet
        if (( eth != KUK_ETHERNET_UNKNOWN)&&( wlan != KUK_WIRELESS_UNKNOWN))
        {
            if ( wlan == KUK_WIRELESS_NONE)
            {
                if ( eth == KUK_ETHERNET_NONE)
                {
                    pArticle[9] = '0';
                }else
                {
                    pArticle[9] = '1';
                }
            }
            if ( wlan == KUK_WIRELESS_HD_SPB228)
            {
                if ( eth == KUK_ETHERNET_NONE)
                {
                    pArticle[9] = '2';
                }else
                {
                    pArticle[9] = '3';
                }
            }
            if ( wlan == KUK_WIRELESS_SILEX_SXPCEAC2)
            {
                if ( eth == KUK_ETHERNET_NONE)
                {
                    pArticle[9] = '4';
                }else
                {
                    pArticle[9] = '5';
                }
            }
        }
        
        switch( kuk_GetBootStorage())
        {
            case KUK_BOOTSTORAGE_SDCARD:    pArticle[10]= '0'; break;
            case KUK_BOOTSTORAGE_EMMC4GB:   pArticle[10]= '1'; break;
            case KUK_BOOTSTORAGE_EMMC8GB:   pArticle[10]= '2'; break;
            case KUK_BOOTSTORAGE_EMMC16GB:  pArticle[10]= '3'; break;
            case KUK_BOOTSTORAGE_EMMC32GB:  pArticle[10]= '4'; break;
            default:                        pArticle[10]= 'x'; break;
        }        
        pArticle[11]= '.';     
        pArticle[12]= 'H';     
        pArticle[13]= 'x';     
        pArticle[14]= 'x';     
        pArticle[15]= 0;
        ret = 16;   
    }
    if (( module == KUK_MODULE_SBCSOM8MINI)||( module == KUK_MODULE_SBCSOM8NANO))
    {
        int eth, wlan;
        if ( module == KUK_MODULE_SBCSOM8MINI)
        {   // Mini
            pArticle[0] = '6';
            pArticle[1] = '4';
            // Get Processor Skew:
            if ( speed <= 1600000000)   
            {   // industrial
                switch( cpu)
                {
                    case MXC_CPU_IMX8MMSL:  pArticle[2] = '1';  break;
                    case MXC_CPU_IMX8MMS:   pArticle[2] = '3';  break;
                    case MXC_CPU_IMX8MMDL:  pArticle[2] = '5';  break;
                    case MXC_CPU_IMX8MMD:   pArticle[2] = '7';  break;
                    case MXC_CPU_IMX8MML:   pArticle[2] = '9';  break;
                    case MXC_CPU_IMX8MM:    pArticle[2] = 'B';  break;
                    default:                pArticle[2] = 'x';  break;
                }
            }else
            {   // consumer
                switch( cpu)
                {
                    case MXC_CPU_IMX8MMSL:  pArticle[2] = '0';  break;
                    case MXC_CPU_IMX8MMS:   pArticle[2] = '2';  break;
                    case MXC_CPU_IMX8MMDL:  pArticle[2] = '4';  break;
                    case MXC_CPU_IMX8MMD:   pArticle[2] = '6';  break;
                    case MXC_CPU_IMX8MML:   pArticle[2] = '8';  break;
                    case MXC_CPU_IMX8MM:    pArticle[2] = 'A';  break;
                    default:                pArticle[2] = 'x';  break;            
                }
            }
        }else
        {   // Nano:
            pArticle[0] = '6';
            pArticle[1] = '4';        
            // Get Processor Skew:
            if ( speed <= 1400000000)   
            {   // industrial
                switch( cpu)
                {
                    case MXC_CPU_IMX8MNSL:  pArticle[2] = 'D';  break;
                    case MXC_CPU_IMX8MNS:   pArticle[2] = 'C';  break;
                    case MXC_CPU_IMX8MNDL:  pArticle[2] = 'E';  break;
                    case MXC_CPU_IMX8MND:   pArticle[2] = 'D';  break;
                    case MXC_CPU_IMX8MNL:   pArticle[2] = 'C';  break;
                    case MXC_CPU_IMX8MN:    pArticle[2] = 'E';  break;
                    default:                pArticle[2] = 'x';  break;
                }
            }else
            {   // consumer
                switch( cpu)
                {
                    case MXC_CPU_IMX8MNSL:  pArticle[2] = 'C';  break;
                    case MXC_CPU_IMX8MNS:   pArticle[2] = 'E';  break;
                    case MXC_CPU_IMX8MNDL:  pArticle[2] = 'D';  break;
                    case MXC_CPU_IMX8MND:   pArticle[2] = 'C';  break;
                    case MXC_CPU_IMX8MNL:   pArticle[2] = 'E';  break;
                    case MXC_CPU_IMX8MN:    pArticle[2] = 'D';  break;
                    default:                pArticle[2] = 'x';  break;            
                }
            }
        }
        
        switch ( ramsize)
        {
            case KUK_RAMSIZE_512MB: pArticle[3] = '0';  break;
            case KUK_RAMSIZE_1GB:   pArticle[3] = '1';  break;
            case KUK_RAMSIZE_2GB:   pArticle[3] = '2';  break;
            case KUK_RAMSIZE_3GB:   pArticle[3] = '3';  break;
            case KUK_RAMSIZE_4GB:   pArticle[3] = '4';  break;
            case KUK_RAMSIZE_8GB:   pArticle[3] = '8';  break;
            default:                pArticle[3] = 'x';  break;
        }
        if ( pcbrev == KUK_PCBREV_UNKNOWN)
            pArticle[4] = 'x';
        else
            pArticle[4] = pcbrev + '0';
        
        pArticle[5] = '.';

        switch( kuk_GetTemperatureRange())
        {
            case KUK_TEMP_COMMERCIAL_0_70:      pArticle[6] = 'C';  break;
            case KUK_TEMP_EXTENDED_m25_85:      pArticle[6] = 'E';  break;
            case KUK_TEMP_INDUSTRIAL_m40_85:    pArticle[6] = 'I';  break;
            default:                            pArticle[6] = 'x';  break;
        }      

        pArticle[7] = 'x';
        pArticle[8] = 'x';    
        eth = kuk_GetPeripheral( KUK_PERIPHERAL_ETHERNET);
        wlan = kuk_GetPeripheral( KUK_PERIPHERAL_WIRELESS);
        pArticle[9] = 'x';      // Ethernet
        if (( eth != KUK_ETHERNET_UNKNOWN)&&( wlan != KUK_WIRELESS_UNKNOWN))
        {
            if ( wlan == KUK_WIRELESS_NONE)
            {
                if ( eth == KUK_ETHERNET_NONE)
                {
                    pArticle[9] = '0';
                }else
                {
                    pArticle[9] = '1';
                }
            }
            if ( wlan == KUK_WIRELESS_HD_SPB228)
            {
                if ( eth == KUK_ETHERNET_NONE)
                {
                    pArticle[9] = '2';
                }else
                {
                    pArticle[9] = '3';
                }
            }
            if ( wlan == KUK_WIRELESS_SILEX_SXPCEAC2)
            {
                if ( eth == KUK_ETHERNET_NONE)
                {
                    pArticle[9] = '4';
                }else
                {
                    pArticle[9] = '5';
                }
            }
        }
        
        switch( kuk_GetBootStorage())
        {
            case KUK_BOOTSTORAGE_SDCARD:    pArticle[10]= '0'; break;
            case KUK_BOOTSTORAGE_EMMC4GB:   pArticle[10]= '1'; break;
            case KUK_BOOTSTORAGE_EMMC8GB:   pArticle[10]= '2'; break;
            case KUK_BOOTSTORAGE_EMMC16GB:  pArticle[10]= '3'; break;
            case KUK_BOOTSTORAGE_EMMC32GB:  pArticle[10]= '4'; break;
            default:                        pArticle[10]= 'x'; break;
        }        
        pArticle[11]= '.';     
        pArticle[12]= 'H';     
        pArticle[13]= 'x';     
        pArticle[14]= 'x';     
        pArticle[15]= 0;
        ret = 16;   
    }
    if (( module == KUK_MODULE_MYON2)||( module == KUK_MODULE_MYON2NANO))
    {
        if ( module == KUK_MODULE_MYON2)
        {   // Mini
            pArticle[0] = '6';
            pArticle[1] = '0';
            // Get Processor Skew:
            if ( speed <= 1600000000)   
            {   // industrial
                switch( cpu)
                {
                    case MXC_CPU_IMX8MMSL:  pArticle[2] = '1';  break;
                    case MXC_CPU_IMX8MMS:   pArticle[2] = '3';  break;
                    case MXC_CPU_IMX8MMDL:  pArticle[2] = '5';  break;
                    case MXC_CPU_IMX8MMD:   pArticle[2] = '7';  break;
                    case MXC_CPU_IMX8MML:   pArticle[2] = '9';  break;
                    case MXC_CPU_IMX8MM:    pArticle[2] = 'B';  break;
                    default:                pArticle[2] = 'x';  break;
                }
            }else
            {   // consumer
                switch( cpu)
                {
                    case MXC_CPU_IMX8MMSL:  pArticle[2] = '0';  break;
                    case MXC_CPU_IMX8MMS:   pArticle[2] = '2';  break;
                    case MXC_CPU_IMX8MMDL:  pArticle[2] = '4';  break;
                    case MXC_CPU_IMX8MMD:   pArticle[2] = '6';  break;
                    case MXC_CPU_IMX8MML:   pArticle[2] = '8';  break;
                    case MXC_CPU_IMX8MM:    pArticle[2] = 'A';  break;
                    default:                pArticle[2] = 'x';  break;            
                }
            }        
        }else
        {   // Nano
            pArticle[0] = 'x';
            pArticle[1] = 'x';        
            // Get Processor Skew:
            pArticle[2] = 'x';
        }
        switch ( ramsize)
        {
            case KUK_RAMSIZE_512MB: pArticle[3] = '0';  break;
            case KUK_RAMSIZE_1GB:   pArticle[3] = '1';  break;
            case KUK_RAMSIZE_2GB:   pArticle[3] = '2';  break;
            case KUK_RAMSIZE_3GB:   pArticle[3] = '3';  break;
            case KUK_RAMSIZE_4GB:   pArticle[3] = '4';  break;
            case KUK_RAMSIZE_8GB:   pArticle[3] = '8';  break;
            default:                pArticle[3] = 'x';  break;
        }
        if ( pcbrev == KUK_PCBREV_UNKNOWN)
            pArticle[4] = 'x';
        else
            pArticle[4] = pcbrev + '0';
        
        pArticle[5] = '.';

        switch( kuk_GetTemperatureRange())
        {
            case KUK_TEMP_COMMERCIAL_0_70:      pArticle[6] = 'C';  break;
            case KUK_TEMP_EXTENDED_m25_85:      pArticle[6] = 'E';  break;
            case KUK_TEMP_INDUSTRIAL_m40_85:    pArticle[6] = 'I';  break;
            default:                            pArticle[6] = 'x';  break;
        }

        switch( kuk_GetPeripheral( KUK_PERIPHERAL_IOVOLTAGE))
        {
            case KUK_IOVOLTAGE_1V8:             pArticle[7] = '0';  break;
            case KUK_IOVOLTAGE_3V3:             pArticle[7] = '1';  break;
            default:                            pArticle[7] = 'x';  break;
        }

        // Periherals: Ethernet, Audio
        switch( kuk_GetPeripheral( KUK_PERIPHERAL_AUDIO))
        {
            case KUK_AUDIO_NONE:                pArticle[8] = '0';  break;
            case KUK_AUDIO_UNKNOWN:             pArticle[8] = 'x';  break;
            default:                            pArticle[8] = '1';  break;
        }

        switch( kuk_GetPeripheral( KUK_PERIPHERAL_LVDS))
        {
            case KUK_LVDS_NONE:                 pArticle[9] = '0';  break;
            case KUK_LVDS_SN65DSI83:            pArticle[9] = '1';  break;
            default:                            pArticle[9] = 'x';  break;
        }

        switch( kuk_GetBootStorage())
        {
            case KUK_BOOTSTORAGE_SDCARD:    pArticle[10]= '0'; break;
            case KUK_BOOTSTORAGE_EMMC4GB:   pArticle[10]= '1'; break;
            case KUK_BOOTSTORAGE_EMMC8GB:   pArticle[10]= '2'; break;
            case KUK_BOOTSTORAGE_EMMC16GB:  pArticle[10]= '3'; break;
            case KUK_BOOTSTORAGE_EMMC32GB:  pArticle[10]= '4'; break;
            default:                        pArticle[10]= 'x'; break;
        }        

        pArticle[11]= '.';     
        pArticle[12]= 'H';     
        pArticle[13]= 'x';     
        pArticle[14]= 'x';     
        pArticle[15]= 0;
        ret = 16;   
    }
    return ret;
}

const char* cModuleName[] = {
    "Unknown", "Trizeps VIII", "Trizeps VIII Mini", "Trizeps VIII Nano", "Myon II", "Myon II Nano", "SBCSOM", "SBCSOM Nano", "Tanaro"
};
const char* cStore[] = {
    "?", "uSD-card", "eMMC", "4GB eMMC", "8GB eMMC", "16GB eMMC", "32GB eMMC"
};
int kuk_GetDescription( char *pDescription, int maxsize )
{
    const char industrial[] = "industrial";
    const char consumer[] = "consumer";
    int module;
    int ramsize;
    int pcbrev;
    unsigned int speed; 
    unsigned int cpu; 
    int cnt = 0;
    char str_ramsize[8];
    char str_cpu[48];
    char str_rev[8];
    int store;

    module  = kuk_GetModule();
    ramsize = kuk_GetRAMSize(); 
    pcbrev  = kuk_GetPCBrevision();
    speed   = get_cpu_speed_grade_hz()/1000000;
    cpu     = get_cpu_type(); 

    switch ( kuk_GetBootStorage())
    {
        case KUK_BOOTSTORAGE_SDCARD:    store = 1;  break;
        case KUK_BOOTSTORAGE_EMMCxGB:   store = 2;  break;
        case KUK_BOOTSTORAGE_EMMC4GB:   store = 3;  break;
        case KUK_BOOTSTORAGE_EMMC8GB:   store = 4;  break;
        case KUK_BOOTSTORAGE_EMMC16GB:  store = 5;  break;
        case KUK_BOOTSTORAGE_EMMC32GB:  store = 6;  break;
        default:                        store = 0;  break;
    }

    if (( pcbrev >= KUK_PCBREV_V1R1)&&( pcbrev <= KUK_PCBREV_V1R3))
    {
        snprintf(&str_rev[0], sizeof(str_rev), "V1R%d", 1 + pcbrev - KUK_PCBREV_V1R1);
    }else
    {
        str_rev[0] = 0;
    }
    
    if ( ramsize == KUK_RAMSIZE_512MB)
    {
        strcpy( &str_ramsize[0], "512MB");
    }else
    if (( ramsize >= KUK_RAMSIZE_1GB)&&( ramsize <= KUK_RAMSIZE_8GB))
    {
        snprintf(&str_ramsize[0], sizeof(str_ramsize), "%dGB", ramsize);
    }else
    {
        strcpy( &str_ramsize[0], "? MB");
    }

    if ( module == KUK_MODULE_TRIZEPS8)
    {
        const char* grade;        
        if ( speed <= 1300)
        {
            grade = industrial;
        }else
        {
            grade = consumer;
        }
                
        switch( cpu)
        {
            case MXC_CPU_IMX8MD:    snprintf(&str_cpu[0], sizeof(str_cpu), "i.MX 8M Dual %dMHz (%s)"        , speed, grade);  break;
            case MXC_CPU_IMX8MQL:   snprintf(&str_cpu[0], sizeof(str_cpu), "i.MX 8M Dual-Lite %dMHz (%s)"   , speed, grade);  break;
            case MXC_CPU_IMX8MQ:    snprintf(&str_cpu[0], sizeof(str_cpu), "i.MX 8M Quad %dMHz (%s)"        , speed, grade);  break;
            default:                snprintf(&str_cpu[0], sizeof(str_cpu), "i.MX 8M %dMHz (%s)"             , speed, grade);  break;
        }
    }else
    if (( module == KUK_MODULE_TRIZEPS8MINI)||( module == KUK_MODULE_MYON2))
    {
        const char* grade;
        if ( speed <= 1600)
        {
            grade = industrial;
        }else
        {
            grade = consumer;
        }
        switch( cpu)
        {
            case MXC_CPU_IMX8MMSL:  snprintf(&str_cpu[0], sizeof(str_cpu), "i.MX 8M Mini Solo-Lite %dMHz (%s)"  , speed, grade);  break;
            case MXC_CPU_IMX8MMS:   snprintf(&str_cpu[0], sizeof(str_cpu), "i.MX 8M Mini Solo %dMHz (%s)"       , speed, grade);  break;
            case MXC_CPU_IMX8MMDL:  snprintf(&str_cpu[0], sizeof(str_cpu), "i.MX 8M Mini Dual-Lite %dMHz (%s)"  , speed, grade);  break;
            case MXC_CPU_IMX8MMD:   snprintf(&str_cpu[0], sizeof(str_cpu), "i.MX 8M Mini Dual %dMHz (%s)"       , speed, grade);  break;
            case MXC_CPU_IMX8MML:   snprintf(&str_cpu[0], sizeof(str_cpu), "i.MX 8M Mini Quad-Lite %dMHz (%s)"  , speed, grade);  break;
            case MXC_CPU_IMX8MM:    snprintf(&str_cpu[0], sizeof(str_cpu), "i.MX 8M Mini Quad %dMHz (%s)"       , speed, grade);  break;
            default:                snprintf(&str_cpu[0], sizeof(str_cpu), "i.MX 8M Mini %dMHz (%s)"            , speed, grade);  break;
        }
    }else
    if (( module == KUK_MODULE_TRIZEPS8NANO)||( module == KUK_MODULE_MYON2NANO))
    {
        snprintf( &str_cpu[0], sizeof(str_cpu), "i.MX 8M Nano %dMHz", speed );
    }

    if ( module >= GUF_MODULE_TANARO)
    {
        cnt += snprintf(&pDescription[cnt], maxsize - cnt, "Garz&Fricke GmbH %s %s running a %s with %s,%dbit RAM booting from %s.\n",
                cModuleName[ module], str_rev, str_cpu, str_ramsize, kuk_GetRAMWidth(), cStore[ store]
            );
    }else{
        cnt += snprintf(&pDescription[cnt], maxsize - cnt, "Keith&Koep GmbH %s %s running a %s with %s,%dbit RAM booting from %s.\n",
                cModuleName[ module], str_rev, str_cpu, str_ramsize, kuk_GetRAMWidth(), cStore[ store]
            );
    }

    switch( kuk_GetTemperatureRange())
    {
        case KUK_TEMP_EXTENDED_m25_85:      cnt += snprintf(&pDescription[cnt], maxsize - cnt, "Extended Temperature-Range\n"); break;
        case KUK_TEMP_INDUSTRIAL_m40_85:    cnt += snprintf(&pDescription[cnt], maxsize - cnt, "Industrial Temperature-Range\n"); break;
        case KUK_TEMP_COMMERCIAL_0_70:      cnt += snprintf(&pDescription[cnt], maxsize - cnt, "Commercial Temperature-Range\n"); break;
        default:                            
            break;
    }

    if (( module == KUK_MODULE_TRIZEPS8)||( module == KUK_MODULE_TRIZEPS8MINI)||( module == KUK_MODULE_TRIZEPS8NANO))
    {
        switch( kuk_GetPeripheral( KUK_PERIPHERAL_MCU))
        {
            case KUK_MCU_NONE:              cnt += snprintf(&pDescription[cnt], maxsize - cnt, "No MCU\n");    break; 
            case KUK_MCU_MKV10Z32VFM7:      cnt += snprintf(&pDescription[cnt], maxsize - cnt, "MCU MKV10Z32VFM7\n");    break; 
            case KUK_MCU_MKV11Z64VFM7:      cnt += snprintf(&pDescription[cnt], maxsize - cnt, "MCU MKV11Z64VFM7\n");    break; 
            default:    break;
        }
        switch ( kuk_GetPeripheral( KUK_PERIPHERAL_LVDS))
        {
            case KUK_LVDS_NONE:             cnt += snprintf(&pDescription[cnt], maxsize - cnt, "No LVDS\n");  break;
            case KUK_LVDS_SN65DSI83:        cnt += snprintf(&pDescription[cnt], maxsize - cnt, "LVDS (single)\n");  break;
            case KUK_LVDS_SN65DSI85:        cnt += snprintf(&pDescription[cnt], maxsize - cnt, "Dual-LVDS\n"); break;
            default:    break;
        }
        switch ( kuk_GetPeripheral( KUK_PERIPHERAL_FPGA))
        {
            case KUK_FPGA_NONE:             cnt += snprintf(&pDescription[cnt], maxsize - cnt, "No FPGA\n");    break;
            case KUK_FPGA_LCMXO3LF_1300E:   cnt += snprintf(&pDescription[cnt], maxsize - cnt, "FPGA LCMXO3LF 1300E\n");  break;
            case KUK_FPGA_LCMXO3LF_2100E:   cnt += snprintf(&pDescription[cnt], maxsize - cnt, "FPGA LCMXO3LF 2100E\n");  break;
            case KUK_FPGA_LCMXO3LF_4300E:   cnt += snprintf(&pDescription[cnt], maxsize - cnt, "FPGA LCMXO3LF 4300E\n");  break;
            case KUK_FPGA_LCMXO3L_1300E:    cnt += snprintf(&pDescription[cnt], maxsize - cnt, "FPGA LCMXO3L 1300E\n");  break;
            case KUK_FPGA_LCMXO3L_2100E:    cnt += snprintf(&pDescription[cnt], maxsize - cnt, "FPGA LCMXO3L 2100E\n");  break;
            case KUK_FPGA_LCMXO3L_4300E:    cnt += snprintf(&pDescription[cnt], maxsize - cnt, "FPGA LCMXO3L 4300E\n");  break;
            default:    break;
        }
        switch(kuk_GetPeripheral( KUK_PERIPHERAL_ETHERNET))
        {
            case KUK_ETHERNET_NONE:         cnt += snprintf(&pDescription[cnt], maxsize - cnt, "No Ethernet\n");    break; 
            case KUK_ETHERNET_AR8031:       cnt += snprintf(&pDescription[cnt], maxsize - cnt, "Ethernet AR8031\n");    break; 
            default:    break;
        }
        switch( kuk_GetPeripheral( KUK_PERIPHERAL_WIRELESS))
        {
            case KUK_WIRELESS_NONE:             cnt += snprintf(&pDescription[cnt], maxsize - cnt, "No Wireless\n");    break;
            case KUK_WIRELESS_LAIRD_SU60:       cnt += snprintf(&pDescription[cnt], maxsize - cnt, "Wireless LAIRD SU60\n");  break;
            case KUK_WIRELESS_LAIRD_ST60:       cnt += snprintf(&pDescription[cnt], maxsize - cnt, "Wireless LAIRD ST60\n");  break;
            case KUK_WIRELESS_HD_SPB228:        cnt += snprintf(&pDescription[cnt], maxsize - cnt, "Wireless HD SPB228\n");  break;
            case KUK_WIRELESS_SILEX_SXPCEAC2:   cnt += snprintf(&pDescription[cnt], maxsize - cnt, "Wireless Silex SX-PCEAC2\n");  break;
            default:    break;
        }        
    }else
    if (( module == KUK_MODULE_MYON2)||( module == KUK_MODULE_MYON2NANO))
    {
        switch ( kuk_GetPeripheral( KUK_PERIPHERAL_LVDS))
        {
            case KUK_LVDS_NONE:             cnt += snprintf(&pDescription[cnt], maxsize - cnt, "Mipi\n");  break;
            case KUK_LVDS_SN65DSI83:        cnt += snprintf(&pDescription[cnt], maxsize - cnt, "LVDS\n");  break;
            default:    break;
        }
        switch( kuk_GetPeripheral( KUK_PERIPHERAL_IOVOLTAGE))
        {
            case KUK_IOVOLTAGE_1V8:         cnt += snprintf(&pDescription[cnt], maxsize - cnt, "IO-Voltage 1.8V\n");  break;
            case KUK_IOVOLTAGE_3V3:         cnt += snprintf(&pDescription[cnt], maxsize - cnt, "IO-Voltage 3.3V\n");  break;
            default:    break;
        }        
    }
    
    switch( kuk_GetPeripheral( KUK_PERIPHERAL_AUDIO))
    {
        case KUK_AUDIO_NONE:        cnt += snprintf(&pDescription[cnt], maxsize - cnt, "No Audio\n");  break;
        case KUK_AUDIO_WM8983:      cnt += snprintf(&pDescription[cnt], maxsize - cnt, "Audio WM8983\n");  break;
        case KUK_AUDIO_WM8978:      cnt += snprintf(&pDescription[cnt], maxsize - cnt, "Audio WM8978\n");  break;
        case KUK_AUDIO_TAS2552:     cnt += snprintf(&pDescription[cnt], maxsize - cnt, "Audio TAS2552\n");  break;
        default:    break;
    }

    return cnt;
}


