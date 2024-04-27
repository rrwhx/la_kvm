#include <assert.h>
#include <elf.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/kvm.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

// static inline long long nano_second(void) {
//     struct timespec _t;
//     clock_gettime(CLOCK_REALTIME, &_t);
//     return _t.tv_sec * 1000000000ll + _t.tv_nsec;
// }

#define UART_LCR_DLAB 0x80 /* Divisor latch access bit */

#define UART_IER_MSI 0x08  /* Enable Modem status interrupt */
#define UART_IER_RLSI 0x04 /* Enable receiver line status interrupt */
#define UART_IER_THRI 0x02 /* Enable Transmitter holding register int. */
#define UART_IER_RDI 0x01  /* Enable receiver data interrupt */

#define UART_IIR_NO_INT 0x01 /* No interrupts pending */
#define UART_IIR_ID 0x06     /* Mask for the interrupt ID */

#define UART_IIR_MSI 0x00  /* Modem status interrupt */
#define UART_IIR_THRI 0x02 /* Transmitter holding register empty */
#define UART_IIR_RDI 0x04  /* Receiver data interrupt */
#define UART_IIR_RLSI 0x06 /* Receiver line status interrupt */
#define UART_IIR_CTI 0x0C  /* Character Timeout Indication */

#define UART_IIR_FENF 0x80 /* Fifo enabled, but not functioning */
#define UART_IIR_FE 0xC0   /* Fifo enabled */

/*
 * These are the definitions for the Modem Control Register
 */
#define UART_MCR_LOOP 0x10 /* Enable loopback test mode */
#define UART_MCR_OUT2 0x08 /* Out2 complement */
#define UART_MCR_OUT1 0x04 /* Out1 complement */
#define UART_MCR_RTS 0x02  /* RTS complement */
#define UART_MCR_DTR 0x01  /* DTR complement */

/*
 * These are the definitions for the Modem Status Register
 */
#define UART_MSR_DCD 0x80       /* Data Carrier Detect */
#define UART_MSR_RI 0x40        /* Ring Indicator */
#define UART_MSR_DSR 0x20       /* Data Set Ready */
#define UART_MSR_CTS 0x10       /* Clear to Send */
#define UART_MSR_DDCD 0x08      /* Delta DCD */
#define UART_MSR_TERI 0x04      /* Trailing edge ring indicator */
#define UART_MSR_DDSR 0x02      /* Delta DSR */
#define UART_MSR_DCTS 0x01      /* Delta CTS */
#define UART_MSR_ANY_DELTA 0x0F /* Any of the delta bits! */

#define UART_LSR_TEMT 0x40    /* Transmitter empty */
#define UART_LSR_THRE 0x20    /* Transmit-hold-register empty */
#define UART_LSR_BI 0x10      /* Break interrupt indicator */
#define UART_LSR_FE 0x08      /* Frame error indicator */
#define UART_LSR_PE 0x04      /* Parity error indicator */
#define UART_LSR_OE 0x02      /* Overrun error indicator */
#define UART_LSR_DR 0x01      /* Receiver data ready */
#define UART_LSR_INT_ANY 0x1E /* Any of the lsr-interrupt-triggering status bits */

/* Interrupt trigger levels. The byte-counts are for 16550A - in newer UARTs the
 * byte-count for each ITL is higher. */

#define UART_FCR_ITL_1 0x00 /* 1 byte ITL */
#define UART_FCR_ITL_2 0x40 /* 4 bytes ITL */
#define UART_FCR_ITL_3 0x80 /* 8 bytes ITL */
#define UART_FCR_ITL_4 0xC0 /* 14 bytes ITL */

#define UART_FCR_DMS 0x08 /* DMA Mode Select */
#define UART_FCR_XFR 0x04 /* XMIT Fifo Reset */
#define UART_FCR_RFR 0x02 /* RCVR Fifo Reset */
#define UART_FCR_FE 0x01  /* FIFO Enable */

#define MAX_XMIT_RETRY 4

struct SerialState {
    // DeviceState parent;

    uint16_t divider;
    uint8_t rbr; /* receive register */
    uint8_t thr; /* transmit holding register */
    uint8_t tsr; /* transmit shift register */
    uint8_t ier;
    uint8_t iir; /* read only */
    uint8_t lcr;
    uint8_t mcr;
    uint8_t lsr; /* read only */
    uint8_t msr; /* read only */
    uint8_t scr;
    uint8_t fcr;
    uint8_t fcr_vmstate; /* we can't write directly this value
                            it has side effects */
    /* NOTE: this hidden state is necessary for tx irq generation as
       it can be reset while reading iir */
    int thr_ipending;
    // qemu_irq irq;
    // CharBackend chr;
    int last_break_enable;
    // uint32_t baudbase;
    // uint32_t tsr_retry;
    // guint watch_tag;
    // bool wakeup;

    // /* Time when the last byte was successfully sent out of the tsr */
    // uint64_t last_xmit_ts;
    // Fifo8 recv_fifo;
    // Fifo8 xmit_fifo;
    // /* Interrupt trigger level for recv_fifo */
    // uint8_t recv_fifo_itl;

    // QEMUtimer_counter *fifo_timeout_timer_counter;
    // int timeout_ipending;           /* timeout interrupt pending state */

    // uint64_t char_transmit_time;    /* time to transmit a char in ticks */
    // int poll_msl;

    // QEMUtimer_counter *modem_status_poll;
    // MemoryRegion io;
};
typedef struct SerialState SerialState;

SerialState s;

char input = 'x';
bool input_vaild;

void try_read() {
    if (!input_vaild) {
        if (read(STDIN_FILENO, &input, 1) == 1) {
            input_vaild = true;
        }
    }
}

uint64_t serial_ioport_read(void* opaque, long addr, unsigned size) {
    try_read();
    uint32_t ret = 0;
    switch (addr) {
    case 0:
        if (s.lcr & UART_LCR_DLAB) {
            ret = (s.divider) & 0Xff;
        } else {
            if (!input_vaild) {
                fprintf(stderr, "lxy: %s:%d %s serial read, while input is empty\n", __FILE__,__LINE__,__func__);
            }
            ret = input;
            input_vaild = false;
        }
        break;
    case 1:
        if (s.lcr & UART_LCR_DLAB) {
            ret = (s.divider >> 8) & 0Xff;
        } else {
            ret = s.ier;
        }
        break;
    case 2:
        ret = 0;
        if (input_vaild) {
            s.iir = UART_IIR_RDI;
        } else if ((s.iir & UART_IIR_ID) == UART_IIR_THRI) {
            s.thr_ipending = 0;
        }
        break;
    case 3:
        fprintf(stderr, "serial_ioport_read, addr:%lx, data:%x, size:%d\n", addr, ret, size);
        break;
    case 4:
        fprintf(stderr, "serial_ioport_read, addr:%lx, data:%x, size:%d\n", addr, ret, size);
        break;
    case 5:
        ret = s.lsr;
        // dr ready
        if (input_vaild) {
            ret |= 1;
        }
        break;
    case 6:
        ret = UART_MSR_DCD | UART_MSR_DSR | UART_MSR_CTS;
        break;
    case 7:
        fprintf(stderr, "serial_ioport_read, addr:%lx, data:%x, size:%d\n", addr, ret, size);
        break;
    default:
        assert(0);
        break;
    }
    // fprintf(stderr, "%ld, serial_ioport_read, addr:%lx, data:%x, size:%d\n",
    // nano_second(), addr, ret, size);
    return ret;
}
void serial_ioport_write(void* opaque, long addr, uint64_t val, unsigned size) {
    try_read();
    switch (addr) {
    case 0:
        if (s.lcr & UART_LCR_DLAB) {
            s.divider = (s.divider & 0xff00) | (val & 0xff);
        } else {
            fprintf(stderr, "%c", (char)val);
            fflush(stderr);
            s.lsr |= (UART_LSR_TEMT | UART_LSR_THRE);
            if (input_vaild) {
                s.iir |= UART_IIR_RDI;
                s.thr_ipending = false;
            } else {
                s.iir |= UART_IIR_THRI;
                s.thr_ipending = true;
            }
        }
        break;
    case 1:
        if (s.lcr & UART_LCR_DLAB) {
            s.divider = (s.divider & 0xff) | ((val << 8) & 0xff);
        } else {
            // uint8_t changed = (s.ier ^ val) & 0x0f;
            // if (changed) {
            //     fprintf(stderr, "serial ier changed\n");
            // }
            s.ier = val & 0x0f;
        }
        break;
    case 2: {
        uint8_t changed = (s.fcr ^ val) & 0xff;
        if (changed) {
            fprintf(stderr, "serial ier changed\n");
        }
        if (val & UART_FCR_RFR) {
            fprintf(stderr, "serial fcr RCVR Fifo Reset\n");
        }
        if (val & UART_FCR_XFR) {
            fprintf(stderr, "serial fcr XMIT Fifo Reset\n");
        }
        s.fcr = val & 0xC9;
    } break;
    case 3:
        s.lcr = val;
        s.last_break_enable = (val >> 6) & 1;
        break;
    case 4: {
        // int old_mcr = s.mcr;
        s.mcr = val & 0x1f;
        if (val & UART_MCR_LOOP)
            break;
    } break;
    case 5:
        fprintf(stderr, "serial_ioport_write, addr:%lx, data:%lx, size:%d\n", addr, val, size);
        break;
    case 6:
        fprintf(stderr, "serial_ioport_write, addr:%lx, data:%lx, size:%d\n", addr, val, size);
        break;
    case 7:
        fprintf(stderr, "serial_ioport_write, addr:%lx, data:%lx, size:%d\n", addr, val, size);
        break;
    default:
        assert(0);
        break;
    }
    // fprintf(stderr, "serial_ioport_write, addr:%lx, data:%lx, size:%d\n",
    // addr, val, size);
}

static void sigaction_entry(int signal, siginfo_t* si, void* arg) {
    // ucontext_t* c = (ucontext_t*)arg;
    printf("signal:%d, at address %p\n", signal, si->si_addr);
    return;
}

static void setup_signal(void) {
    struct sigaction sa;
    memset(&sa, 0, sizeof(struct sigaction));
    sigemptyset(&sa.sa_mask);
    sa.sa_sigaction = sigaction_entry;
    sa.sa_flags = SA_SIGINFO;
    if (sigaction(SIGINT, &sa, NULL)) {
        printf("signal %d:%s register failed\n", SIGINT, strsignal(SIGINT));
        exit(1);
    }
}

#if 1
#define lsassert(cond)                                                                                             \
    do {                                                                                                           \
        if (!(cond)) {                                                                                             \
            fprintf(stderr, "\033[31m assertion failed in <%s> %s:%d \033[m\n", __FUNCTION__, __FILE__, __LINE__); \
            abort();                                                                                               \
        }                                                                                                          \
    } while (0)

#define lsassertm(cond, ...)                                                                                     \
    do {                                                                                                         \
        if (!(cond)) {                                                                                           \
            fprintf(stderr, "\033[31m assertion failed in <%s> %s:%d \033[m", __FUNCTION__, __FILE__, __LINE__); \
            fprintf(stderr, __VA_ARGS__);                                                                        \
            abort();                                                                                             \
        }                                                                                                        \
    } while (0)

#else
#define lsassert(cond) ((void)0)
#define lsassertm(cond, ...) ((void)0)
#endif

const char* const loongarch_r_alias[32] = {
    "zer", "ra", "tp", "sp", "a0", "a1", "a2", "a3", "a4", "a5", "a6", "a7", "t0", "t1", "t2", "t3", "t4", "t5", "t6", "t7", "t8", "r21", "fp", "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7", "s8",
};

static void show_register(struct kvm_regs *env) {
    fprintf(stderr, "pc:0x%llx\n", env->pc);
    for (int i = 0; i < 32; i++) {
        fprintf(stderr, "r%02d/%-3s:%016llx    ", i, loongarch_r_alias[i], env->gpr[i]);
        if ((i + 1) % 4 == 0) {
            fprintf(stderr, "\n");
        }
    }
}

/* Loongarch KVM register ids */
#define LOONGARCH_CSR_32(_R, _S) (KVM_REG_LOONGARCH_CSR | KVM_REG_SIZE_U32 | (8 * (_R) + (_S)))

#define LOONGARCH_CSR_64(_R, _S) (KVM_REG_LOONGARCH_CSR | KVM_REG_SIZE_U64 | (8 * (_R) + (_S)))

#define KVM_LOONGARCH_CSR_CRMD 0
#define KVM_LOONGARCH_CSR_PRMD 1
#define KVM_LOONGARCH_CSR_EUEN 2
#define KVM_LOONGARCH_CSR_MISC 3
#define KVM_LOONGARCH_CSR_ECFG 4
#define KVM_LOONGARCH_CSR_ESTAT 5
#define KVM_LOONGARCH_CSR_EPC 6
#define KVM_LOONGARCH_CSR_BADV 7
#define KVM_LOONGARCH_CSR_BADI 8
#define KVM_LOONGARCH_CSR_EBASE 0xc
#define KVM_LOONGARCH_CSR_TLBIDX 0x10
#define KVM_LOONGARCH_CSR_TLBHI 0x11
#define KVM_LOONGARCH_CSR_TLBLO0 0x12
#define KVM_LOONGARCH_CSR_TLBLO1 0x13
#define KVM_LOONGARCH_CSR_GTLBC 0x15
#define KVM_LOONGARCH_CSR_TRGP 0x16
#define KVM_LOONGARCH_CSR_ASID 0x18
#define KVM_LOONGARCH_CSR_PGDL 0x19
#define KVM_LOONGARCH_CSR_PGDH 0x1a
#define KVM_LOONGARCH_CSR_PGD 0x1b
#define KVM_LOONGARCH_CSR_PWCTL0 0x1c
#define KVM_LOONGARCH_CSR_PWCTL1 0x1d
#define KVM_LOONGARCH_CSR_STLBPS 0x1e
#define KVM_LOONGARCH_CSR_RVACFG 0x1f
#define KVM_LOONGARCH_CSR_CPUNUM 0x20
#define KVM_LOONGARCH_CSR_PRCFG1 0x21
#define KVM_LOONGARCH_CSR_PRCFG2 0x22
#define KVM_LOONGARCH_CSR_PRCFG3 0x23
#define KVM_LOONGARCH_CSR_KSCRATCH0 0x30
#define KVM_LOONGARCH_CSR_KSCRATCH1 0x31
#define KVM_LOONGARCH_CSR_KSCRATCH2 0x32
#define KVM_LOONGARCH_CSR_KSCRATCH3 0x33
#define KVM_LOONGARCH_CSR_KSCRATCH4 0x34
#define KVM_LOONGARCH_CSR_KSCRATCH5 0x35
#define KVM_LOONGARCH_CSR_KSCRATCH6 0x36
#define KVM_LOONGARCH_CSR_KSCRATCH7 0x37
#define KVM_LOONGARCH_CSR_KSCRATCH8 0x38
#define KVM_LOONGARCH_CSR_TIMERID 0x40
#define KVM_LOONGARCH_CSR_TIMERCFG 0x41
#define KVM_LOONGARCH_CSR_TIMERTICK 0x42
#define KVM_LOONGARCH_CSR_TIMEROFFSET 0x43
#define KVM_LOONGARCH_CSR_GSTAT 0x50
#define KVM_LOONGARCH_CSR_GCFG 0x51
#define KVM_LOONGARCH_CSR_GINTC 0x52
#define KVM_LOONGARCH_CSR_GCNTC 0x53
#define KVM_LOONGARCH_CSR_LLBCTL 0x60
#define KVM_LOONGARCH_CSR_IMPCTL1 0x80
#define KVM_LOONGARCH_CSR_IMPCTL2 0x81
#define KVM_LOONGARCH_CSR_GNMI 0x82
#define KVM_LOONGARCH_CSR_TLBREBASE 0x88
#define KVM_LOONGARCH_CSR_TLBRBADV 0x89
#define KVM_LOONGARCH_CSR_TLBREPC 0x8a
#define KVM_LOONGARCH_CSR_TLBRSAVE 0x8b
#define KVM_LOONGARCH_CSR_TLBRELO0 0x8c
#define KVM_LOONGARCH_CSR_TLBRELO1 0x8d
#define KVM_LOONGARCH_CSR_TLBREHI 0x8e
#define KVM_LOONGARCH_CSR_TLBRPRMD 0x8f
#define KVM_LOONGARCH_CSR_ERRCTL 0x90
#define KVM_LOONGARCH_CSR_ERRINFO1 0x91
#define KVM_LOONGARCH_CSR_ERRINFO2 0x92
#define KVM_LOONGARCH_CSR_ERREBASE 0x93
#define KVM_LOONGARCH_CSR_ERREPC 0x94
#define KVM_LOONGARCH_CSR_ERRSAVE 0x95
#define KVM_LOONGARCH_CSR_CTAG 0x98
#define KVM_LOONGARCH_CSR_MCSR0 0xc0
#define KVM_LOONGARCH_CSR_MCSR1 0xc1
#define KVM_LOONGARCH_CSR_MCSR2 0xc2
#define KVM_LOONGARCH_CSR_MCSR3 0xc3
#define KVM_LOONGARCH_CSR_MCSR8 0xc8
#define KVM_LOONGARCH_CSR_MCSR9 0xc9
#define KVM_LOONGARCH_CSR_MCSR10 0xca
#define KVM_LOONGARCH_CSR_MCSR24 0xf0
#define KVM_LOONGARCH_CSR_UCAWIN 0x100
#define KVM_LOONGARCH_CSR_UCAWIN0_LO 0x102
#define KVM_LOONGARCH_CSR_UCAWIN0_HI 0x103
#define KVM_LOONGARCH_CSR_UCAWIN1_LO 0x104
#define KVM_LOONGARCH_CSR_UCAWIN1_HI 0x105
#define KVM_LOONGARCH_CSR_UCAWIN2_LO 0x106
#define KVM_LOONGARCH_CSR_UCAWIN2_HI 0x107
#define KVM_LOONGARCH_CSR_UCAWIN3_LO 0x108
#define KVM_LOONGARCH_CSR_UCAWIN3_HI 0x109
#define KVM_LOONGARCH_CSR_DMWIN0 0x180
#define KVM_LOONGARCH_CSR_DMWIN1 0x181
#define KVM_LOONGARCH_CSR_DMWIN2 0x182
#define KVM_LOONGARCH_CSR_DMWIN3 0x183
#define KVM_LOONGARCH_CSR_PERF0_EVENT 0x200
#define KVM_LOONGARCH_CSR_PERF0_COUNT 0x201
#define KVM_LOONGARCH_CSR_PERF1_EVENT 0x202
#define KVM_LOONGARCH_CSR_PERF1_COUNT 0x203
#define KVM_LOONGARCH_CSR_PERF2_EVENT 0x204
#define KVM_LOONGARCH_CSR_PERF2_COUNT 0x205
#define KVM_LOONGARCH_CSR_PERF3_EVENT 0x206
#define KVM_LOONGARCH_CSR_PERF3_COUNT 0x207
#define KVM_LOONGARCH_CSR_DEBUG 0x500
#define KVM_LOONGARCH_CSR_DEPC 0x501
#define KVM_LOONGARCH_CSR_DESAVE 0x502

#define KVM_CSR_CRMD LOONGARCH_CSR_64(0, 0)
#define KVM_CSR_PRMD LOONGARCH_CSR_64(1, 0)
#define KVM_CSR_EUEN LOONGARCH_CSR_64(2, 0)
#define KVM_CSR_MISC LOONGARCH_CSR_64(3, 0)
#define KVM_CSR_ECFG LOONGARCH_CSR_64(4, 0)
#define KVM_CSR_ESTAT LOONGARCH_CSR_64(5, 0)
#define KVM_CSR_EPC LOONGARCH_CSR_64(6, 0)
#define KVM_CSR_BADV LOONGARCH_CSR_64(7, 0)
#define KVM_CSR_BADI LOONGARCH_CSR_64(8, 0)
#define KVM_CSR_EBASE LOONGARCH_CSR_64(0xc, 0)
#define KVM_CSR_TLBIDX LOONGARCH_CSR_64(0x10, 0)
#define KVM_CSR_TLBHI LOONGARCH_CSR_64(0x11, 0)
#define KVM_CSR_TLBLO0 LOONGARCH_CSR_64(0x12, 0)
#define KVM_CSR_TLBLO1 LOONGARCH_CSR_64(0x13, 0)
#define KVM_CSR_GTLBC LOONGARCH_CSR_64(0x15, 0)
#define KVM_CSR_TRGP LOONGARCH_CSR_64(0x16, 0)
#define KVM_CSR_ASID LOONGARCH_CSR_64(0x18, 0)
#define KVM_CSR_PGDL LOONGARCH_CSR_64(0x19, 0)
#define KVM_CSR_PGDH LOONGARCH_CSR_64(0x1a, 0)
#define KVM_CSR_PGD LOONGARCH_CSR_64(0x1b, 0)
#define KVM_CSR_PWCTL0 LOONGARCH_CSR_64(0x1c, 0)
#define KVM_CSR_PWCTL1 LOONGARCH_CSR_64(0x1d, 0)
#define KVM_CSR_STLBPS LOONGARCH_CSR_64(0x1e, 0)
#define KVM_CSR_RVACFG LOONGARCH_CSR_64(0x1f, 0)
#define KVM_CSR_CPUNUM LOONGARCH_CSR_64(0x20, 0)
#define KVM_CSR_PRCFG1 LOONGARCH_CSR_64(0x21, 0)
#define KVM_CSR_PRCFG2 LOONGARCH_CSR_64(0x22, 0)
#define KVM_CSR_PRCFG3 LOONGARCH_CSR_64(0x23, 0)
#define KVM_CSR_KSCRATCH0 LOONGARCH_CSR_64(0x30, 0)
#define KVM_CSR_KSCRATCH1 LOONGARCH_CSR_64(0x31, 0)
#define KVM_CSR_KSCRATCH2 LOONGARCH_CSR_64(0x32, 0)
#define KVM_CSR_KSCRATCH3 LOONGARCH_CSR_64(0x33, 0)
#define KVM_CSR_KSCRATCH4 LOONGARCH_CSR_64(0x34, 0)
#define KVM_CSR_KSCRATCH5 LOONGARCH_CSR_64(0x35, 0)
#define KVM_CSR_KSCRATCH6 LOONGARCH_CSR_64(0x36, 0)
#define KVM_CSR_KSCRATCH7 LOONGARCH_CSR_64(0x37, 0)
#define KVM_CSR_KSCRATCH8 LOONGARCH_CSR_64(0x38, 0)
#define KVM_CSR_TIMERID LOONGARCH_CSR_64(0x40, 0)
#define KVM_CSR_TIMERCFG LOONGARCH_CSR_64(0x41, 0)
#define KVM_CSR_TIMERTICK LOONGARCH_CSR_64(0x42, 0)
#define KVM_CSR_TIMEROFFSET LOONGARCH_CSR_64(0x43, 0)
#define KVM_CSR_GSTAT LOONGARCH_CSR_64(0x50, 0)
#define KVM_CSR_GCFG LOONGARCH_CSR_64(0x51, 0)
#define KVM_CSR_GINTC LOONGARCH_CSR_64(0x52, 0)
#define KVM_CSR_GCNTC LOONGARCH_CSR_64(0x53, 0)
#define KVM_CSR_LLBCTL LOONGARCH_CSR_64(0x60, 0)
#define KVM_CSR_IMPCTL1 LOONGARCH_CSR_64(0x80, 0)
#define KVM_CSR_IMPCTL2 LOONGARCH_CSR_64(0x81, 0)
#define KVM_CSR_GNMI LOONGARCH_CSR_64(0x82, 0)
#define KVM_CSR_TLBREBASE LOONGARCH_CSR_64(0x88, 0)
#define KVM_CSR_TLBRBADV LOONGARCH_CSR_64(0x89, 0)
#define KVM_CSR_TLBREPC LOONGARCH_CSR_64(0x8a, 0)
#define KVM_CSR_TLBRSAVE LOONGARCH_CSR_64(0x8b, 0)
#define KVM_CSR_TLBRELO0 LOONGARCH_CSR_64(0x8c, 0)
#define KVM_CSR_TLBRELO1 LOONGARCH_CSR_64(0x8d, 0)
#define KVM_CSR_TLBREHI LOONGARCH_CSR_64(0x8e, 0)
#define KVM_CSR_TLBRPRMD LOONGARCH_CSR_64(0x8f, 0)
#define KVM_CSR_ERRCTL LOONGARCH_CSR_64(0x90, 0)
#define KVM_CSR_ERRINFO1 LOONGARCH_CSR_64(0x91, 0)
#define KVM_CSR_ERRINFO2 LOONGARCH_CSR_64(0x92, 0)
#define KVM_CSR_ERREBASE LOONGARCH_CSR_64(0x93, 0)
#define KVM_CSR_ERREPC LOONGARCH_CSR_64(0x94, 0)
#define KVM_CSR_ERRSAVE LOONGARCH_CSR_64(0x95, 0)
#define KVM_CSR_CTAG LOONGARCH_CSR_64(0x98, 0)
#define KVM_CSR_MCSR0 LOONGARCH_CSR_64(0xc0, 0)
#define KVM_CSR_MCSR1 LOONGARCH_CSR_64(0xc1, 0)
#define KVM_CSR_MCSR2 LOONGARCH_CSR_64(0xc2, 0)
#define KVM_CSR_MCSR3 LOONGARCH_CSR_64(0xc3, 0)
#define KVM_CSR_MCSR8 LOONGARCH_CSR_64(0xc8, 0)
#define KVM_CSR_MCSR9 LOONGARCH_CSR_64(0xc9, 0)
#define KVM_CSR_MCSR10 LOONGARCH_CSR_64(0xca, 0)
#define KVM_CSR_MCSR24 LOONGARCH_CSR_64(0xf0, 0)
#define KVM_CSR_UCWIN LOONGARCH_CSR_64(0x100, 0)
#define KVM_CSR_UCWIN0_LO LOONGARCH_CSR_64(0x102, 0)
#define KVM_CSR_UCWIN0_HI LOONGARCH_CSR_64(0x103, 0)
#define KVM_CSR_UCWIN1_LO LOONGARCH_CSR_64(0x104, 0)
#define KVM_CSR_UCWIN1_HI LOONGARCH_CSR_64(0x105, 0)
#define KVM_CSR_UCWIN2_LO LOONGARCH_CSR_64(0x106, 0)
#define KVM_CSR_UCWIN2_HI LOONGARCH_CSR_64(0x107, 0)
#define KVM_CSR_UCWIN3_LO LOONGARCH_CSR_64(0x108, 0)
#define KVM_CSR_UCWIN3_HI LOONGARCH_CSR_64(0x109, 0)
#define KVM_CSR_DMWIN0 LOONGARCH_CSR_64(0x180, 0)
#define KVM_CSR_DMWIN1 LOONGARCH_CSR_64(0x181, 0)
#define KVM_CSR_DMWIN2 LOONGARCH_CSR_64(0x182, 0)
#define KVM_CSR_DMWIN3 LOONGARCH_CSR_64(0x183, 0)
#define KVM_CSR_PERF0_EVENT LOONGARCH_CSR_64(0x200, 0)
#define KVM_CSR_PERF0_COUNT LOONGARCH_CSR_64(0x201, 0)
#define KVM_CSR_PERF1_EVENT LOONGARCH_CSR_64(0x202, 0)
#define KVM_CSR_PERF1_COUNT LOONGARCH_CSR_64(0x203, 0)
#define KVM_CSR_PERF2_EVENT LOONGARCH_CSR_64(0x204, 0)
#define KVM_CSR_PERF2_COUNT LOONGARCH_CSR_64(0x205, 0)
#define KVM_CSR_PERF3_EVENT LOONGARCH_CSR_64(0x206, 0)
#define KVM_CSR_PERF3_COUNT LOONGARCH_CSR_64(0x207, 0)
#define KVM_CSR_DEBUG LOONGARCH_CSR_64(0x500, 0)
#define KVM_CSR_DEPC LOONGARCH_CSR_64(0x501, 0)
#define KVM_CSR_DESAVE LOONGARCH_CSR_64(0x502, 0)

static void show_csr(int vcpu_fd) {
    struct kvm_one_reg csr;
    int err;
    uint64_t csr_value;

#define DUMP_KVM_CSR(csrid, name)                    \
    do {                                             \
        csr.addr = (uint64_t) & (csr_value);         \
        csr.id = csrid;                              \
        err = ioctl(vcpu_fd, KVM_GET_ONE_REG, &csr); \
        lsassert(err >= 0);                          \
        printf("%-12s 0x%lx\n", name, csr_value);    \
    } while (0)

    DUMP_KVM_CSR(KVM_CSR_CRMD, "CRMD");
    DUMP_KVM_CSR(KVM_CSR_ASID, "ASID");
    DUMP_KVM_CSR(KVM_CSR_BADV, "BADV");
    DUMP_KVM_CSR(KVM_CSR_BADI, "BADI");
    DUMP_KVM_CSR(KVM_CSR_EPC, "EPC");
    DUMP_KVM_CSR(KVM_CSR_ESTAT, "ESTAT");
    DUMP_KVM_CSR(KVM_CSR_STLBPS, "STLBPS");
    DUMP_KVM_CSR(KVM_CSR_TLBRBADV, "TLBR BADV");
    DUMP_KVM_CSR(KVM_CSR_TLBREPC, "TLBR EPC");
    DUMP_KVM_CSR(KVM_CSR_TLBREHI, "TLBR EHI");
    DUMP_KVM_CSR(KVM_CSR_TLBRELO0, "TLBR ELO0");
    DUMP_KVM_CSR(KVM_CSR_TLBRELO1, "TLBR ELO1");
    DUMP_KVM_CSR(KVM_CSR_KSCRATCH0, "KS0");
    DUMP_KVM_CSR(KVM_CSR_KSCRATCH1, "KS1");
    DUMP_KVM_CSR(KVM_CSR_KSCRATCH2, "KS2");
    DUMP_KVM_CSR(KVM_CSR_KSCRATCH3, "KS3");
    DUMP_KVM_CSR(KVM_CSR_KSCRATCH4, "KS4");
    DUMP_KVM_CSR(KVM_CSR_KSCRATCH5, "KS5");
    DUMP_KVM_CSR(KVM_CSR_KSCRATCH6, "KS6");
    DUMP_KVM_CSR(KVM_CSR_KSCRATCH7, "KS7");
    DUMP_KVM_CSR(KVM_CSR_KSCRATCH8, "KS8");
    DUMP_KVM_CSR(KVM_CSR_PWCTL0, "PWCL");
    DUMP_KVM_CSR(KVM_CSR_PWCTL1, "PWCH");
}

#include <linux/const.h>

#define SZ_1				0x00000001
#define SZ_2				0x00000002
#define SZ_4				0x00000004
#define SZ_8				0x00000008
#define SZ_16				0x00000010
#define SZ_32				0x00000020
#define SZ_64				0x00000040
#define SZ_128				0x00000080
#define SZ_256				0x00000100
#define SZ_512				0x00000200

#define SZ_1K				0x00000400
#define SZ_2K				0x00000800
#define SZ_4K				0x00001000
#define SZ_8K				0x00002000
#define SZ_16K				0x00004000
#define SZ_32K				0x00008000
#define SZ_64K				0x00010000
#define SZ_128K				0x00020000
#define SZ_256K				0x00040000
#define SZ_512K				0x00080000

#define SZ_1M				0x00100000
#define SZ_2M				0x00200000
#define SZ_4M				0x00400000
#define SZ_8M				0x00800000
#define SZ_16M				0x01000000
#define SZ_32M				0x02000000
#define SZ_64M				0x04000000
#define SZ_128M				0x08000000
#define SZ_256M				0x10000000
#define SZ_512M				0x20000000

#define SZ_1G				0x40000000
#define SZ_2G				0x80000000

#define SZ_4G				_AC(0x100000000, ULL)
#define SZ_8G				_AC(0x200000000, ULL)
#define SZ_16G				_AC(0x400000000, ULL)
#define SZ_32G				_AC(0x800000000, ULL)

void* ram;

static char* alloc_ram(uint64_t ram_size) {
    void *start = mmap(NULL, ram_size + SZ_2G, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    lsassert(start != MAP_FAILED);
    void *part1 = mmap(start, SZ_256M, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
    lsassert(part1 != MAP_FAILED);
    void *part2 = mmap(start + SZ_2G, ram_size - SZ_256M, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
    lsassert(part2 != MAP_FAILED);
    return part1;
}

#define elfhdr Elf64_Ehdr
#define elf_shdr Elf64_Shdr
#define elf_phdr Elf64_Phdr

bool load_elf(const char* filename, uint64_t* entry_addr) {
    int size, i;
    uint64_t mem_size, file_size;
    uint8_t e_ident[EI_NIDENT];
    uint8_t* data = NULL;
    int ret = 1;
    elfhdr ehdr;
    elf_phdr *phdr = NULL, *ph;
    int fd = open(filename, O_RDONLY);
    if (fd < 0) {
        perror(filename);
        goto fail;
    }

    if (read(fd, e_ident, sizeof(e_ident)) != sizeof(e_ident))
        goto fail;
    if (e_ident[0] != ELFMAG0 || e_ident[1] != ELFMAG1 || e_ident[2] != ELFMAG2 || e_ident[3] != ELFMAG3) {
        lsassertm(0, "%s is not an elf\n", filename);
    }
    lsassert(e_ident[EI_CLASS] == ELFCLASS64);
    lseek(fd, 0, SEEK_SET);

    if (read(fd, &ehdr, sizeof(ehdr)) != sizeof(ehdr))
        goto fail;

    *entry_addr = ehdr.e_entry;

    size = ehdr.e_phnum * sizeof(phdr[0]);
    if (lseek(fd, ehdr.e_phoff, SEEK_SET) != ehdr.e_phoff) {
        goto fail;
    }
    phdr = (elf_phdr*)malloc(size);
    if (!phdr)
        goto fail;

    if (read(fd, phdr, size) != size)
        goto fail;

    for (i = 0; i < ehdr.e_phnum; i++) {
        ph = &phdr[i];
        if (ph->p_type == PT_LOAD) {
            mem_size = ph->p_memsz;   /* Size of the ROM */
            file_size = ph->p_filesz; /* Size of the allocated data */
            data = (uint8_t*)malloc(file_size);
            if (ph->p_filesz > 0) {
                if (lseek(fd, ph->p_offset, SEEK_SET) < 0) {
                    goto fail;
                }
                if (read(fd, data, file_size) != file_size) {
                    goto fail;
                }
                memcpy(ram + (ph->p_paddr & 0xfffffff), data, file_size);
                fprintf(stderr, "%lx, file_size:%lx mem_size:%lx\n", ph->p_paddr, file_size, mem_size);
            }
        }
    }

fail:
    close(fd);
    return ret;
}

void get_regs(int vcpu_fd, struct kvm_regs* regs) {
    int ret;
    ret = ioctl(vcpu_fd, KVM_GET_REGS, regs);
    lsassertm(ret == 0, "%s\n", strerror(errno));
}

void usage(void) {
    fprintf(stderr, "la_emu_kernel -m n[G] -k kernel\n");
    exit(EXIT_SUCCESS);
}

int main(int argc, char **argv) {
    char* kernel_filename = "vmlinux";
    uint64_t ram_size = SZ_16G;

    int c;
    while ((c = getopt(argc, argv, "+m:k:d:D:")) != -1) {
        switch (c) {
        case 'm':
            ram_size = atol(optarg) << 30;
            break;
        case 'k':
            kernel_filename = optarg;
            break;
        // case 'd':
        //     handle_logmask(optarg);
        //     break;
        // case 'D':
        //     handle_logfile(optarg);
        //     break;
        case '?':
            usage();
            return 1;
        default:
            abort();
        }
    }

    // set no echo
    // struct termios term;
    // tcgetattr(STDIN_FILENO, &term);

    // term.c_lflag &= ~ECHO;
    // tcsetattr(STDIN_FILENO, 0, &term);

    fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);

    setup_signal();
    ram = alloc_ram(ram_size);

    uint64_t entry_addr;
    load_elf(kernel_filename, &entry_addr);

    int ret;
    int sys_fd = open("/dev/kvm", O_RDWR);
    printf("sys_fd:%d\n", sys_fd);
    lsassertm(sys_fd > 0, "%s\n", strerror(errno));
    ret = ioctl(sys_fd, KVM_GET_API_VERSION, 0);
    lsassert(ret == KVM_API_VERSION);
    printf("KVM_API_VERSION:%d\n", KVM_API_VERSION);
    int vm_fd = ioctl(sys_fd, KVM_CREATE_VM, 0);
    lsassert(vm_fd >= 0);
    printf("vm_fd:%d\n", vm_fd);
    int vcpu_mmap_size = ioctl(sys_fd, KVM_GET_VCPU_MMAP_SIZE, 0);
    lsassert(vcpu_mmap_size > 0);
    printf("vcpu_mmap_size:%d\n", vcpu_mmap_size);

    // hvcl debug
    // *(uint32_t*) (low_mem) = 0x002b8005;

    // mmio example
    //    0:    14220004    lu12i.w  $r4,69632(0x11000)
    //    4:    28c00084    ld.d     $r4,$r4,0
    // *(uint32_t*) (low_mem + 0x1000000) = 0x14220004;
    // *(uint32_t*) (low_mem + 0x1000004) = 0x28c00084;

    struct kvm_userspace_memory_region low_memory_region = {
        .slot = 0,
        .flags = 0,
        .guest_phys_addr = 0,
        .memory_size = SZ_256M,
        .userspace_addr = (__u64)ram,
    };
    ret = ioctl(vm_fd, KVM_SET_USER_MEMORY_REGION, &low_memory_region);
    lsassertm(ret == 0, "%s\n", strerror(errno));

    struct kvm_userspace_memory_region high_memory_region = {
        .slot = 1,
        .flags = 0,
        .guest_phys_addr = (SZ_2G + SZ_256M),
        .memory_size = (ram_size - SZ_256M),
        .userspace_addr = (__u64)(ram + SZ_2G + SZ_256M),
    };
    ret = ioctl(vm_fd, KVM_SET_USER_MEMORY_REGION, &high_memory_region);
    lsassertm(ret == 0, "%s\n", strerror(errno));

    int vcpu_fd = ioctl(vm_fd, KVM_CREATE_VCPU, 0);
    lsassertm(vcpu_fd >= 0, "%s\n", strerror(errno));

    struct kvm_run* kvm_run = mmap(NULL, vcpu_mmap_size, PROT_READ | PROT_WRITE, MAP_SHARED, vcpu_fd, 0);
    lsassertm(kvm_run != MAP_FAILED, "%s\n", strerror(errno));

    struct kvm_regs regs;
    get_regs(vcpu_fd, &regs);
    regs.pc = entry_addr;
    ret = ioctl(vcpu_fd, KVM_SET_REGS, &regs);
    lsassert(ret == 0);

    show_register(&regs);
    show_csr(vcpu_fd);

    // uint64_t val = 0x8;
    // struct kvm_one_reg crmd = {
    //     .id = KVM_CSR_CRMD,
    // };
    // crmd.addr = (__u64)&val;
    // ret = ioctl(vcpu_fd, KVM_SET_ONE_REG, &crmd);
    // lsassertm(ret == 0, "%s\n", strerror(errno));

    while (1) {
        ret = ioctl(vcpu_fd, KVM_RUN, 0);
        if (ret < 0) {
            if (errno == EINTR) {
                fprintf(stderr, "lxy: %s:%d %s Interrupted system call KVM_RUN\n", __FILE__, __LINE__, __FUNCTION__);
            } else {
                fprintf(stderr, "lxy: %s:%d %s %s\n", __FILE__, __LINE__, __FUNCTION__, strerror(errno));
                return 0;
            }
        }

        get_regs(vcpu_fd, &regs);
        // fprintf(stderr, "lxy: %s:%d %s pc:%lx\n", __FILE__,__LINE__,__FUNCTION__,
        // regs.pc); show_register(&regs); show_csr(vcpu_fd);

        switch (kvm_run->exit_reason) {
        case KVM_EXIT_DEBUG:
            printf("exit_reason:%d KVM_EXIT_DEBUG\n", kvm_run->exit_reason);
            printf("debug_era:%llx\n", kvm_run->debug.arch.era);
            printf("debug_exception:%x\n", kvm_run->debug.arch.exception);
            break;
        case KVM_EXIT_INTERNAL_ERROR:
            printf("exit_reason:%d KVM_EXIT_INTERNAL_ERROR\n", kvm_run->exit_reason);
            printf("%d\n", kvm_run->internal.suberror);
            break;
        case KVM_EXIT_MMIO: {
#define UART_BASE 0x1fe001e0
#define UART_END 0x1fe001e7
            uint64_t mmio_addr = kvm_run->mmio.phys_addr;
            if (mmio_addr == 0x1fe002e0) {
                if (kvm_run->mmio.is_write) {
                    fprintf(stderr, "%c", kvm_run->mmio.data[0]);
                    fflush(stderr);
                } else {
                    fprintf(stderr, "mmio read at 0x1fe002e0\n");
                }
            } else if (mmio_addr >= UART_BASE && mmio_addr <= UART_END) {
                if (kvm_run->mmio.is_write) {
                    serial_ioport_write(NULL, mmio_addr - UART_BASE, kvm_run->mmio.data[0], 1);
                } else {
                    uint64_t data = serial_ioport_read(NULL, mmio_addr - UART_BASE, 1);
                    kvm_run->mmio.data[0] = data;
                }
            } else if (mmio_addr == 0x100d0014) {
                if (kvm_run->mmio.is_write) {
                    uint32_t data = *(uint32_t *)kvm_run->mmio.data;
                    fprintf(stderr,
                            "lxy: %s:%d %s poweroff@100d0014 data:%x "
                            "pc:%llx ra:%llx\n",
                            __FILE__, __LINE__, __FUNCTION__, data, regs.pc, regs.gpr[1]);
                    if ((data & 0x3c00) == 0x3c00) {
                        return 0;
                    }
                } else {
                    *(uint32_t *)kvm_run->mmio.data = 0;
                }
            } else {
                fprintf(stderr, "lxy: %s:%d %s unknown mmio %s at %lx\n", __FILE__, __LINE__, __FUNCTION__, kvm_run->mmio.is_write ? "write" : "read", mmio_addr);
            }
        } break;
        case KVM_EXIT_INTR:
            printf("exit_reason:%d KVM_EXIT_INTR\n", kvm_run->exit_reason);
            show_register(&regs);
            show_csr(vcpu_fd);
            return 0;
            break;
        default:
            fprintf(stderr, "lxy: %s:%d %s exit_reason:%d\n", __FILE__, __LINE__, __FUNCTION__, kvm_run->exit_reason);
            break;
        }
    }

    return 0;
}