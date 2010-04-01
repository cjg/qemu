/* -*- Mode: C; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/*
 * VMSemaphore PCI device.
 *
 * Author:
 *      Giuseppe Coviello <giuseppe.coviello@uniparthenope.it>
 *
 * Based On: vmsemaphore.c
 *
 * This code is licensed under the GNU GPL v2.
 */

#include <sys/mman.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <semaphore.h>
#include "hw.h"
#include "pci.h"
#include "pc.h"
#include "qemu_socket.h"
#include "qemu-thread.h"

#define PCI_COMMAND_IOACCESS                0x0001
#define PCI_COMMAND_MEMACCESS               0x0002
#define PCI_COMMAND_BUSMASTER               0x0004

#define DEBUG_VMSEMAPHORE

#ifdef DEBUG_VMSEMAPHORE
#define VMSEMAPHORE_DPRINTF(fmt, args...)                   \
    do {printf("VMSEMAPHORE: " fmt, ##args); } while (0)
#else
#define VMSEMAPHORE_DPRINTF(fmt, args...)
#endif

/* Registers */
/* Read Only */
#define VMSEMAPHORE_OPEN_L_REG          0x0
#define VMSEMAPHORE_POST_L_REG          0x20
#define VMSEMAPHORE_WAIT_BEGIN_L_REG    0x40
#define VMSEMAPHORE_WAIT_END_L_REG      0x60
#define VMSEMAPHORE_CLOSE_L_REG         0x80
#define VMSEMAPHORE_INTR_GET_L_REG      0xA0
/* Write Only */
#define VMSEMAPHORE_INTR_SET_L_REG      0xC0
#define VMSEMAPHORE_SET_L_REG           0xE0

typedef struct VMSemaphoreState {
    uint32_t regs_addr;

    void *ioname;
    uint32_t ioname_offset;
    uint32_t ioname_size;

    char *name;

    sem_t *sem;
    int interrupt;
    pthread_mutex_t mutex;

    PCIDevice *pci_dev;
} VMSemaphoreState;

typedef struct PCI_VMSemaphoreState {
    PCIDevice dev;
    VMSemaphoreState vmsemaphore_state;
} PCI_VMSemaphoreState;

char *vmsemaphore_devices[MAX_VMSEMAPHORE_DEVICES];
static int vmsemaphore_count = 0;

static void *vmsemaphore_wait_begin(void *arg)
{
    VMSemaphoreState *s = (VMSemaphoreState *) arg;
    sem_wait(s->sem);
    VMSEMAPHORE_DPRINTF("[%s] Wake up.\n", s->name);
    VMSEMAPHORE_DPRINTF("[%s] Trying to acquire lock.\n", s->name);
    pthread_mutex_lock(&s->mutex);
    s->interrupt = 1;
    qemu_set_irq(s->pci_dev->irq[0], 1);
    return NULL;
}


void vmsemaphore_init(const char * optarg) {
    if(vmsemaphore_count >= MAX_VMSEMAPHORE_DEVICES) {
        VMSEMAPHORE_DPRINTF("cannot enabled another semaphore.\n");
        return;
    }
    vmsemaphore_devices[vmsemaphore_count] = strdup(optarg);
    vmsemaphore_count++;
}

static uint32_t vmsemaphore_regs_readl(void *opaque, target_phys_addr_t addr) {
    VMSemaphoreState * s = opaque;
    int status = 0;
    switch (addr & 0xFF) {
    case VMSEMAPHORE_OPEN_L_REG:
        s->sem = sem_open(s->name, O_CREAT, 00600, 0);
        status = s->sem == SEM_FAILED;
        VMSEMAPHORE_DPRINTF("[%s] Open status: %d\n", s->name, status);
        if(status != 0)
            VMSEMAPHORE_DPRINTF("[%s] Open error: %s\n", s->name, strerror(errno));
        return status;
    case VMSEMAPHORE_POST_L_REG:
        VMSEMAPHORE_DPRINTF("Post.\n");
        status = sem_post(s->sem);
        VMSEMAPHORE_DPRINTF("Post status: %d\n", status);
        return status;
    case VMSEMAPHORE_WAIT_BEGIN_L_REG:
        VMSEMAPHORE_DPRINTF("[%s] Wait.\n", s->name);
        pthread_mutex_lock(&s->mutex);
        s->interrupt = 0;
        pthread_t t;
        pthread_create(&t, NULL, vmsemaphore_wait_begin, s);
        pthread_mutex_unlock(&s->mutex);
        return 0;
    case VMSEMAPHORE_WAIT_END_L_REG:
        s->interrupt = 0;
        qemu_set_irq(s->pci_dev->irq[0], 0);
        pthread_mutex_unlock(&s->mutex);
        return 0;
    case VMSEMAPHORE_CLOSE_L_REG:
        status = sem_close(s->sem);
        VMSEMAPHORE_DPRINTF("Close status: %d\n", status);
        return status;
    case VMSEMAPHORE_INTR_GET_L_REG:
        return s->interrupt;
    }
    VMSEMAPHORE_DPRINTF("reading long from invalid register 0x%x.\n",
                        (uint32_t) addr & 0xFF);
    return 0;
}

static CPUReadMemoryFunc *vmsemaphore_regs_read[3] = {
    NULL,
    NULL,
    vmsemaphore_regs_readl,
};

static void vmsemaphore_regs_writel(void *opaque, target_phys_addr_t addr,
                                    uint32_t uvalue) {
    VMSemaphoreState * s = opaque;
    int value = (int) uvalue;
    int val;
    switch(addr & 0xFF) {
    case VMSEMAPHORE_INTR_SET_L_REG:
        s->interrupt = uvalue;
        qemu_set_irq(s->pci_dev->irq[0], s->interrupt);
        break;
    case VMSEMAPHORE_SET_L_REG:
        sem_getvalue(s->sem, &val);
        VMSEMAPHORE_DPRINTF("Setting semaphore from %u to %u.\n",
                            val, value);
        for(sem_getvalue(s->sem, &val); val > value;
            sem_getvalue(s->sem, &val)) {
            sem_wait(s->sem);
        }
        for(sem_getvalue(s->sem, &val); val < value;
            sem_getvalue(s->sem, &val)) {
            sem_post(s->sem);
        }
        break;
    default:
        VMSEMAPHORE_DPRINTF("writing long to invalid register 0x%x.\n",
                            (uint32_t) addr & 0xFF);
    }
}


static CPUWriteMemoryFunc *vmsemaphore_regs_write[3] = {
    NULL,
    NULL,
    vmsemaphore_regs_writel
};

static void vmsemaphore_region_map(PCIDevice *pci_dev, int region_num,
                                   pcibus_t addr, pcibus_t size, int type) {
    PCI_VMSemaphoreState *d = (PCI_VMSemaphoreState *) pci_dev;
    VMSemaphoreState *s = &d->vmsemaphore_state;

    switch(region_num) {
    case 0:
        VMSEMAPHORE_DPRINTF("Register regs at 0x%x.\n", (unsigned) addr);
        cpu_register_physical_memory(addr, 0x100, s->regs_addr);
        break;
    case 1:
        VMSEMAPHORE_DPRINTF("Buffer ioname at 0x%x.\n",
                            (unsigned) addr);
        cpu_register_physical_memory(addr, s->ioname_size,
                                     s->ioname_offset);
        break;
    }
}

void pci_vmsemaphore_init(PCIBus *bus) {
    int i;
    for(i = 0; i < vmsemaphore_count; i++) {
        PCI_VMSemaphoreState *d;
        VMSemaphoreState *s;
        uint8_t *pci_conf;

        VMSEMAPHORE_DPRINTF("Enabling '%s'.\n", vmsemaphore_devices[i]);

        d = (PCI_VMSemaphoreState *) pci_register_device(bus, "kvm_vmsemaphore",
                                                         sizeof
                                                         (PCI_VMSemaphoreState), -1,
                                                         NULL, NULL);

        if (d == NULL) {
            VMSEMAPHORE_DPRINTF("Can't register pci device.\n");
            return;
        }

        s = &d->vmsemaphore_state;
        s->pci_dev = &d->dev;

        /* Registers */
        s->regs_addr = cpu_register_io_memory(vmsemaphore_regs_read,
                                              vmsemaphore_regs_write, s);

        /* I/O Buffers */
        s->ioname_size = 1024 * 1024; /* FIXME: make it configurable */
        s->ioname_offset = qemu_ram_alloc(s->ioname_size);
        s->ioname = qemu_get_ram_ptr(s->ioname_offset);
        memmove(s->ioname, vmsemaphore_devices[i], strlen(vmsemaphore_devices[i]) + 1);

        /* PCI config */
        pci_conf = d->dev.config;
        pci_config_set_vendor_id(pci_conf, PCI_VENDOR_ID_REDHAT_QUMRANET);
        pci_config_set_device_id(pci_conf, 0x6660);
        pci_config_set_class(pci_conf, PCI_CLASS_OTHERS);
        pci_conf[PCI_HEADER_TYPE] = PCI_HEADER_TYPE_NORMAL; // header_type
        pci_conf[PCI_INTERRUPT_PIN] = 1; // we are going to support interrupts

        /* Regions */
        pci_register_bar(&d->dev, 0, 0x100, PCI_BASE_ADDRESS_SPACE_MEMORY,
                         vmsemaphore_region_map);
        pci_register_bar(&d->dev, 1, s->ioname_size, PCI_BASE_ADDRESS_SPACE_MEMORY,
                         vmsemaphore_region_map);

        s->name = vmsemaphore_devices[i];
        s->interrupt = 0;
        qemu_set_irq(s->pci_dev->irq[0], 0);
        pthread_mutex_init(&s->mutex, NULL);
    }
}

int vmsemaphore_get_buffer_size(void)
{
    return 1024 * 1024 * 2; // FIXME: make it configurable
}


