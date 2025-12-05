/*
** ###################################################################
**
**     Copyright 2023 NXP
**
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**
**     o Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**
**     o Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**
**     o Neither the name of the copyright holder nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**
** ###################################################################
*/

/*==========================================================================*/
/* File containing the implementation of the device ROM access functions.   */
/*==========================================================================*/

/* Includes */

#include "sm.h"
#include "dev_sm.h"

/* Local defines */

#define HANDOVER_BASE    0x2003DC00U
#define HANDOVER_BARKER  0xC0FFEE16U
#define HANDOVER_VER     0x2U
#define HANDOVER_SIZE    0x100U

#define PASSOVER_BASE  0x2003DE00U
#define PASSOVER_TAG   0x504FU
#define PASSOVER_SIZE  0x80U
#define PASSOVER_VER   0x2U

/* ROM handover image flags */
#define ROM_HANDOVER_IMG_CPU(x)    (((x) & 0x00FFU) >> 0U)
#define ROM_HANDOVER_IMG_TYPE(x)   (((x) & 0xFF00U) >> 8U)
#define ROM_HANDOVER_IMG_MSEL(x)   (((x) & 0x00FF0000U) >> 16U)
#define ROM_HANDOVER_IMG_FLAGS(x)  (((x) & 0xFF000000U) >> 24U)

#define ROM_STAGE_SHIFT  24U
#define ROM_STAGE_MASK   0x0F000000UL

#define ROM_CONTAINER_1  0x55CCU
#define ROM_CONTAINER_2  0xAA33U

/* Local types */

/* Local variables */

static uint32_t s_imageIdx = 0U;
static uint64_t s_m7Addr = 0U;
static bool s_m7AddValid = false;

/*--------------------------------------------------------------------------*/
/* Init passover data                                                       */
/*--------------------------------------------------------------------------*/
void DEV_SM_RomInit(void)
{
    printf("DEBUG: mimx95 dev_sm_rom: DEV_SM_RomInit() called, digprog_device_id=0x%X\n",
         OSC24M->DIGPROG_DEVICE_ID);
    /* Silicon rev is Ax? */
    if ((OSC24M->DIGPROG_DEVICE_ID & 0xF0U) == 0x10U)
    {
        uint32_t romPatchVer = DEV_SM_FuseGet(DEV_SM_FUSE_M33_ROM_PATCH_VER);
        printf("DEBUG: mimx95 dev_sm_rom: romPatchVer=0x%X\n", romPatchVer);

        /* Is M7 powered? */
        bool pdMixOn = SRC_MixIsPwrSwitchOn(DEV_SM_PD_M7);
        printf("DEBUG: mimx95 dev_sm_rom: M7 power domain state=%d\n", pdMixOn);

        // only for print out log of M7 reset vector
        if (pdMixOn) {
            uint64_t m7Addr;
            bool m7AddrValid = CPU_ResetVectorGet(DEV_SM_CPU_M7P, &m7Addr);
            printf("DEBUG: mimx95 dev_sm_rom: M7 is powered, valid=%d, m7addr=0x%X_%08X\n",
                m7AddrValid, INT64_H(m7Addr), INT64_L(m7Addr));
        } else {
            printf("DEBUG: mimx95 dev_sm_rom: M7 is not powered\n");
        }

        /* No ROM patch? */
        if (romPatchVer == 0x0U)
        {
            if (pdMixOn)
            {
                /* Load address from reset vector registers */
                s_m7AddValid = CPU_ResetVectorGet(DEV_SM_CPU_M7P,
                    &s_m7Addr);
            }
        }
    }
}

/*--------------------------------------------------------------------------*/
/* Return handover data                                                     */
/*--------------------------------------------------------------------------*/
int32_t DEV_SM_RomHandoverGet(const rom_handover_t **handover)
{
    int32_t status = SM_ERR_SUCCESS;
    const rom_handover_t *ptr = (const rom_handover_t *) HANDOVER_BASE;

    /* Check barker */
    if (ptr->barker != HANDOVER_BARKER)
    {
        status = SM_ERR_NOT_SUPPORTED;
    }

    /* Check version */
    if ((status == SM_ERR_SUCCESS) && (ptr->ver != HANDOVER_VER))
    {
        status = SM_ERR_NOT_SUPPORTED;
    }

    /* Check size */
    if ((status == SM_ERR_SUCCESS) && (ptr->size < sizeof(rom_handover_t)))
    {
        status = SM_ERR_NOT_SUPPORTED;
    }

    /* Return pointer to data */
    if (status == SM_ERR_SUCCESS)
    {
        *handover = ptr;
    }

#ifdef DEBUG_VERBOSE
    // this will be annoying from loopping RomBootImgNGet :)
    printf("DEBUG: mimx95 dev_sm_rom: DEV_SM_RomHandoverGet() status=0x%X from 0x%X\n", status, HANDOVER_BASE);
    if (status == SM_ERR_SUCCESS)
    {
        printf("  barker=0x%X\n", ptr->barker); //0xC0FFEE16 HANDOVER_BARKER
        printf("  ver=0x%X\n", ptr->ver);     // 0x2 HANDOVER_VER
        printf("  size=0x%X\n", ptr->size);   // 0x100
        printf("  num=0x%X\n", ptr->num);     // 0x3
        printf("  flags=0x%X\n", ptr->flags); // 0x113
        printf("  img[0]= cpu:0x%X addr:0x%X_%08X, flags:0x%X\n",
            ptr->img[0].cpu, INT64_H(ptr->img[0].addr), INT64_L(ptr->img[0].addr),
            ptr->img[0].flags); // cpu=1
        printf("  img[1]= cpu:0x%X addr:0x%X_%08X, flags:0x%X\n",
            ptr->img[1].cpu, INT64_H(ptr->img[1].addr), INT64_L(ptr->img[1].addr),
            ptr->img[1].flags); // cpu=b?
        printf("  img[2]= cpu:0x%X addr:0x%X_%08X, flags:0x%X\n",
            ptr->img[2].cpu, INT64_H(ptr->img[2].addr), INT64_L(ptr->img[2].addr),
            ptr->img[2].flags); // cpu=2
    }
#endif
    /* Return status */
    return status;
}

/*--------------------------------------------------------------------------*/
/* Return passover data                                                     */
/*--------------------------------------------------------------------------*/
int32_t DEV_SM_RomPassoverGet(const rom_passover_t **passover)
{
    int32_t status = SM_ERR_SUCCESS;
    const rom_passover_t *ptr = (const rom_passover_t *) PASSOVER_BASE;

    /* Check tag */
    if (ptr->tag != PASSOVER_TAG)
    {
        status = SM_ERR_NOT_SUPPORTED;
    }

    /* Check version */
    if ((status == SM_ERR_SUCCESS) && (ptr->ver != PASSOVER_VER))
    {
        status = SM_ERR_NOT_SUPPORTED;
    }

    /* Check size */
    if ((status == SM_ERR_SUCCESS) && (ptr->size < sizeof(rom_passover_t)))
    {
        status = SM_ERR_NOT_SUPPORTED;
    }

    /* Return pointer to data */
    if (status == SM_ERR_SUCCESS)
    {
        *passover = ptr;
    }

#ifdef DEBUG_VERBOSE
    // would be from DEV_SM_Init and LMM_MiscRomPassoverGet mostly
    printf("DEBUG: mimx95 dev_sm_rom: DEV_SM_RomPassoverGet() status=0x%X\n", status);
    if (status == SM_ERR_SUCCESS)
    {
        printf("  size=0x%X\n", ptr->size); // 0x80
        printf("  ver=0x%X\n", ptr->ver);   // 0x2
        printf("  bootMode=0x%X\n", ptr->bootMode); // 0x02
        printf("  cardAddrMode=0x%X\n", ptr->cardAddrMode); // 0x1(because i use SD card)
        printf("  bootStage=0x%X\n", ptr->bootStage); // 0x6
        printf("  imgSetSel=0x%X\n", ptr->imgSetSel); // 0x0
        printf("  romVersion=0x%X\n", ptr->romVersion); // 0x11
        printf("  bootDevType=0x%X\n", ptr->bootDevType); // 0x1(SD/eMMC)
        printf("  devPageSize=0x%X\n", ptr->devPageSize); // 512 bytes(RM Chapter 13.10 PAGE_SIZE_OF_BOOT_DEVICE)
        printf("  cntHeaderOfs=0x%X\n", ptr->cntHeaderOfs); // 0x0
        printf("  imgOfs=0x%X\n", ptr->imgOfs); // 0x8000(RM Chapter 13.15.2.4 1st boot partition offset)
    }
#endif
    /* Return status */
    return status;
}

/*--------------------------------------------------------------------------*/
/* Return first image                                                       */
/*--------------------------------------------------------------------------*/
int32_t DEV_SM_RomBootImg1Get(uint32_t type, uint32_t *cpuId,
    uint64_t *addr, uint32_t *mSel, uint32_t *flags)
{
    s_imageIdx = 0U;

    return DEV_SM_RomBootImgNGet(type, cpuId, addr, mSel, flags);
}

/*--------------------------------------------------------------------------*/
/* Return next image                                                        */
/*--------------------------------------------------------------------------*/
int32_t DEV_SM_RomBootImgNGet(uint32_t type, uint32_t *cpuId,
    uint64_t *addr, uint32_t *mSel, uint32_t *flags)
{
    int32_t status;
    const rom_handover_t *ptr;

    /* Get handover pointer */
    status = DEV_SM_RomHandoverGet(&ptr);

    /* Loop looking for next image type */
    while ((status == SM_ERR_SUCCESS) && (s_imageIdx < ptr->num))
    {
        const rom_bootlist_t *img = &(ptr->img[s_imageIdx]);

        /* Image type found? */
        if (type == ROM_HANDOVER_IMG_TYPE(img->flags))
        {
            break;
        }
        else
        {
            /* Next image */
            s_imageIdx++;
        }
    }

    /* Found? */
    if ((status == SM_ERR_SUCCESS) && (s_imageIdx < ptr->num))
    {
        const rom_bootlist_t *img = &(ptr->img[s_imageIdx]);

        /* Return data */
        *cpuId = ROM_HANDOVER_IMG_CPU(img->flags);
        *mSel = ROM_HANDOVER_IMG_MSEL(img->flags);
        *flags = ROM_HANDOVER_IMG_FLAGS(img->flags);
        *addr = img->addr;
#ifdef DEBUG_VERBOSE
        // I had monitor cmd shows this info, so keep it verbose here
        printf("DEBUG: mimx95 dev_sm_rom: DEV_SM_RomBootImgNGet() imageIdx=%d cpuId=0x%X addr=0x%X_%08X mSel=0x%X flags=0x%X\n",
            s_imageIdx, *cpuId, INT64_H(*addr), INT64_L(*addr), *mSel, *flags);
#endif
        /* Fix for ROM address patch */
        if (s_m7AddValid && (*cpuId == DEV_SM_CPU_M7P))
        {
            *addr = s_m7Addr;
#ifdef DEBUG
            // can patch boot address from s_m7Addr
            printf("DEBUG: mimx95 dev_sm_rom: DEV_SM_RomBootImgNGet() patched M7 addr=0x%X_%08X\n", INT64_H(*addr), INT64_L(*addr));
#endif
        }

        /* Next image on next call */
        s_imageIdx++;
    }
    else
    {
        status = SM_ERR_NOT_FOUND;
    }

    /* Return status */
    return status;
}

/*--------------------------------------------------------------------------*/
/* Get CPU boot data                                                        */
/*--------------------------------------------------------------------------*/
int32_t DEV_SM_RomBootCpuGet(uint32_t cpuId, uint64_t *resetVector,
    uint32_t *mSel, uint32_t *flags)
{
    int32_t status;
    uint32_t cpu;
    uint64_t addr;
    uint32_t sel;
    uint32_t fl;

    /* Get first image */
    status = DEV_SM_RomBootImg1Get(DEV_SM_ROM_IMG_EXEC, &cpu, &addr,
        &sel, &fl);

    /* Loop over images */
    while (status == SM_ERR_SUCCESS)
    {
        /* Found? */
        if (cpuId == cpu)
        {
#ifdef DEBUG
            // NOTE: `cpu` `sel` and `fi` from handover rom img's flags
            printf("DEBUG: mimx95 dev_sm_rom: DEV_SM_RomBootCpuGet() cpuId=0x%X founded\n", cpuId);
            printf("  type=0x%X(DEV_SM_ROM_IMG_EXEC)\n", DEV_SM_ROM_IMG_EXEC);
            printf("  addr=0x%X_%08X\n", INT64_H(addr), INT64_L(addr));
            printf("  mSel=0x%X, flags=0x%X\n", sel, fl);
#endif
            /* Return data */
            *resetVector = addr;
            *mSel = sel;
            *flags = fl;
            break;
        }

        /* Get next image */
        status = DEV_SM_RomBootImgNGet(DEV_SM_ROM_IMG_EXEC, &cpu, &addr,
            &sel, &fl);
    }

    /* Return status */
    return status;
}

/*--------------------------------------------------------------------------*/
/* Set boot stage                                                           */
/*--------------------------------------------------------------------------*/
int32_t DEV_SM_RomStageSet(uint32_t stage)
{
    int32_t status = SM_ERR_SUCCESS;
    uint32_t temp;

    /* Configure stage */
    temp = SRC_GEN->GPR16 & ~ROM_STAGE_MASK;
    SRC_GEN->GPR16 = temp | ((stage << ROM_STAGE_SHIFT) & ROM_STAGE_MASK);
#ifdef DEBUG
    printf("DEBUG: mimx95 dev_sm_rom: DEV_SM_RomStageSet() stage=0x%X, status=0x%X\n", stage, status);
#endif
    /* Return status */
    return status;
}

/*--------------------------------------------------------------------------*/
/* Get boot stage                                                           */
/*--------------------------------------------------------------------------*/
uint32_t DEV_SM_RomStageGet(void)
{
    uint32_t stage = (SRC_GEN->GPR16 & ROM_STAGE_MASK) >> ROM_STAGE_SHIFT;

    /* Return the current stage */
    return stage;
}

/*--------------------------------------------------------------------------*/
/* Set boot container                                                       */
/*--------------------------------------------------------------------------*/
int32_t DEV_SM_RomContainerSet(uint32_t container)
{
    int32_t status = SM_ERR_SUCCESS;

    /* Configure container */
    if (container == 0U)
    {
        SRC_GEN->GPR15 = ROM_CONTAINER_1;
    }
    else if (container == 1U)
    {
        SRC_GEN->GPR15 = ROM_CONTAINER_2;
    }
    else
    {
        status = SM_ERR_INVALID_PARAMETERS;
    }
#ifdef DEBUG
    printf("DEBUG: mimx95 dev_sm_rom: DEV_SM_RomContainerSet() container=0x%X, status=0x%X\n", container, status);
#endif
    /* Return status */
    return status;
}

/*--------------------------------------------------------------------------*/
/* Get boot container                                                       */
/*--------------------------------------------------------------------------*/
uint32_t DEV_SM_RomContainerGet(void)
{
    /* get the current container value */
    uint32_t container = SRC_GEN->GPR15;

    /* return the current container value */
    return container;
}
