/*****************************************************************************
* Product: DPP example, POSIX
* Last Updated for Version: 4.5.02
* Date of the Last Update:  Jul 25, 2012
*
*                    Q u a n t u m     L e a P s
*                    ---------------------------
*                    innovating embedded systems
*
* Copyright (C) 2002-2012 Quantum Leaps, LLC. All rights reserved.
*
* This program is open source software: you can redistribute it and/or
* modify it under the terms of the GNU General Public License as published
* by the Free Software Foundation, either version 2 of the License, or
* (at your option) any later version.
*
* Alternatively, this program may be distributed and modified under the
* terms of Quantum Leaps commercial licenses, which expressly supersede
* the GNU General Public License and are specifically designed for
* licensees interested in retaining the proprietary status of their code.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* Contact information:
* Quantum Leaps Web sites: http://www.quantum-leaps.com
*                          http://www.state-machine.com
* e-mail:                  info@quantum-leaps.com
*****************************************************************************/
#include "qp_port.h"
#include "project.h"
#include "bsp.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>                            /* for memcpy() and memset() */
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

Q_DEFINE_THIS_FILE

uint8_t topScreen;
uint8_t botScreen;

/* Local objects -----------------------------------------------------------*/
static struct termios l_tsav;   /* structure with saved terminal attributes */
static long int l_tickUsec = 10000UL;   /* clock tick in usec (for tv_usec) */

#ifdef Q_SPY
    static uint8_t const l_clock_tick = 0U;
#endif

/*..........................................................................*/
void BSP_setTickRate(uint32_t ticksPerSec) {
    l_tickUsec = 1000000UL / ticksPerSec;
}
/*..........................................................................*/
void BSP_init(void) {
    printf("Ring for posix"
           "\nQEP %s\nQF  %s\n"
           "Press ESC to quit...\n",
           QEP_getVersion(),
           QF_getVersion());

    Q_ALLEGE(QS_INIT(0));
    QS_RESET();
    QS_OBJ_DICTIONARY(&l_clock_tick);   /* must be called *after* QF_init() */
}
/*..........................................................................*/
void BSP_terminate(int result) {
    QF_stop();
    exit(result);
}

/*..........................................................................*/
void QF_onStartup(void) {                               /* startup callback */
    struct termios tio;                     /* modified terminal attributes */

    tcgetattr(0, &l_tsav);          /* save the current terminal attributes */
    tcgetattr(0, &tio);           /* obtain the current terminal attributes */
    tio.c_lflag &= ~(ICANON | ECHO);   /* disable the canonical mode & echo */
    tcsetattr(0, TCSANOW, &tio);                  /* set the new attributes */

    BSP_setTickRate(BSP_TICKS_PER_SEC);         /* set the desired tick rate */
}
/*..........................................................................*/
void QF_onCleanup(void) {                               /* cleanup callback */
    printf("\nBye! Bye!\n");
    tcsetattr(0, TCSANOW, &l_tsav);/* restore the saved terminal attributes */
    QS_EXIT();                                    /* perform the QS cleanup */
    exit(0);
}
/*..........................................................................*/
void QF_onClockTick(void) {
    struct timeval timeout = { 0 };                 /* timeout for select() */
    fd_set con;                          /* FD set representing the console */

    QF_TICK(&l_clock_tick);         /* perform the QF clock tick processing */

    FD_ZERO(&con);
    FD_SET(0, &con);
    /* check if a console input is available, returns immediately */
    if (0 != select(1, &con, 0, 0, &timeout)) {  /* any descriptor set? */
        char ch;
        read(0, &ch, 1);
        if (ch == '\33') {                              /* ESC pressed? */
            BSP_terminate(-1);
        } else {
            printf("%4x\n", ch);
        }
    }
}
/*..........................................................................*/
void QF_onIdle(void) {       /* called with interrupts disabled, see NOTE01 */
#ifdef Q_SPY
    if ((UART0->FR & UART_FR_TXFE) != 0) {                      /* TX done? */
        uint16_t fifo = UART_TXFIFO_DEPTH;       /* max bytes we can accept */
        uint8_t const *block;

        QF_INT_DISABLE();
        block = QS_getBlock(&fifo);    /* try to get next block to transmit */
        QF_INT_ENABLE();

        while (fifo-- != 0) {                    /* any bytes in the block? */
            UART0->DR = *block++;                      /* put into the FIFO */
        }
    }
#elif defined NDEBUG
    /* Put the CPU and peripherals to the low-power mode.
    * you might need to customize the clock management for your application,
    * see the datasheet for your particular Cortex-M3 MCU.
    */

    struct timeval timeout = { 0 };                 /* timeout for select() */
    timeout.tv_usec = l_tickUsec;          /* set the desired tick interval */
    select(0, 0, 0, 0, &timeout);     /* sleep for the desired tick, NOTE05 */

    QF_INT_ENABLE();
#else
    QF_INT_ENABLE();
#endif

    QF_onClockTick();          /* clock tick callback (must call QF_TICK()) */
}
/*..........................................................................*/
void Q_onAssert(char const Q_ROM * const Q_ROM_VAR file, int line) {
    fprintf(stderr, "Assertion failed in %s, line %d", file, line);
    QF_stop();
}

/*--------------------------------------------------------------------------*/
#ifdef Q_SPY                                         /* define QS callbacks */

#include "qspy.h"

static uint8_t l_running;

/*..........................................................................*/
static void *idleThread(void *par) {     /* the expected P-Thread signature */
    (void)par;

    l_running = (uint8_t)1;
    while (l_running) {
        uint16_t nBytes = 256;
        uint8_t const *block;
        struct timeval timeout = { 0 };             /* timeout for select() */

        QF_CRIT_ENTRY(dummy);
        block = QS_getBlock(&nBytes);
        QF_CRIT_EXIT(dummy);

        if (block != (uint8_t *)0) {
            QSPY_parse(block, nBytes);
        }
        timeout.tv_usec = 10000;
        select(0, 0, 0, 0, &timeout);                  /* sleep for a while */
    }
    return 0;                                             /* return success */
}
/*..........................................................................*/
uint8_t QS_onStartup(void const *arg) {
    static uint8_t qsBuf[4*1024];                 // 4K buffer for Quantum Spy
    QS_initBuf(qsBuf, sizeof(qsBuf));

    QSPY_config((QP_VERSION >> 8),  // version
                QS_OBJ_PTR_SIZE,    // objPtrSize
                QS_FUN_PTR_SIZE,    // funPtrSize
                QS_TIME_SIZE,       // tstampSize
                Q_SIGNAL_SIZE,      // sigSize,
                QF_EVENT_SIZ_SIZE,  // evtSize
                QF_EQUEUE_CTR_SIZE, // queueCtrSize
                QF_MPOOL_CTR_SIZE,  // poolCtrSize
                QF_MPOOL_SIZ_SIZE,  // poolBlkSize
                QF_TIMEEVT_CTR_SIZE,// tevtCtrSize
                (void *)0,          // matFile,
                (void *)0,
                (QSPY_CustParseFun)0); // customized parser function

    QS_FILTER_ON(QS_ALL_RECORDS);

//    QS_FILTER_OFF(QS_QEP_STATE_EMPTY);
//    QS_FILTER_OFF(QS_QEP_STATE_ENTRY);
//    QS_FILTER_OFF(QS_QEP_STATE_EXIT);
//    QS_FILTER_OFF(QS_QEP_STATE_INIT);
//    QS_FILTER_OFF(QS_QEP_INIT_TRAN);
//    QS_FILTER_OFF(QS_QEP_INTERN_TRAN);
//    QS_FILTER_OFF(QS_QEP_TRAN);
//    QS_FILTER_OFF(QS_QEP_IGNORED);
//    QS_FILTER_OFF(QS_QEP_DISPATCH);
//    QS_FILTER_OFF(QS_QEP_UNHANDLED);

    QS_FILTER_OFF(QS_QF_ACTIVE_ADD);
    QS_FILTER_OFF(QS_QF_ACTIVE_REMOVE);
    QS_FILTER_OFF(QS_QF_ACTIVE_SUBSCRIBE);
//    QS_FILTER_OFF(QS_QF_ACTIVE_UNSUBSCRIBE);
    QS_FILTER_OFF(QS_QF_ACTIVE_POST_FIFO);
//    QS_FILTER_OFF(QS_QF_ACTIVE_POST_LIFO);
    QS_FILTER_OFF(QS_QF_ACTIVE_GET);
    QS_FILTER_OFF(QS_QF_ACTIVE_GET_LAST);
    QS_FILTER_OFF(QS_QF_EQUEUE_INIT);
    QS_FILTER_OFF(QS_QF_EQUEUE_POST_FIFO);
    QS_FILTER_OFF(QS_QF_EQUEUE_POST_LIFO);
    QS_FILTER_OFF(QS_QF_EQUEUE_GET);
    QS_FILTER_OFF(QS_QF_EQUEUE_GET_LAST);
    QS_FILTER_OFF(QS_QF_MPOOL_INIT);
    QS_FILTER_OFF(QS_QF_MPOOL_GET);
    QS_FILTER_OFF(QS_QF_MPOOL_PUT);
    QS_FILTER_OFF(QS_QF_PUBLISH);
    QS_FILTER_OFF(QS_QF_NEW);
    QS_FILTER_OFF(QS_QF_GC_ATTEMPT);
    QS_FILTER_OFF(QS_QF_GC);
    QS_FILTER_OFF(QS_QF_TICK);
    QS_FILTER_OFF(QS_QF_TIMEEVT_ARM);
    QS_FILTER_OFF(QS_QF_TIMEEVT_AUTO_DISARM);
    QS_FILTER_OFF(QS_QF_TIMEEVT_DISARM_ATTEMPT);
    QS_FILTER_OFF(QS_QF_TIMEEVT_DISARM);
    QS_FILTER_OFF(QS_QF_TIMEEVT_REARM);
    QS_FILTER_OFF(QS_QF_TIMEEVT_POST);
    QS_FILTER_OFF(QS_QF_CRIT_ENTRY);
    QS_FILTER_OFF(QS_QF_CRIT_EXIT);
    QS_FILTER_OFF(QS_QF_ISR_ENTRY);
    QS_FILTER_OFF(QS_QF_ISR_EXIT);

    QS_RESET();

    {
        pthread_attr_t attr;
        struct sched_param param;
        pthread_t idle;

        /* SCHED_FIFO corresponds to real-time preemptive priority-based
        * scheduler.
        * NOTE: This scheduling policy requires the superuser priviledges
        */
        pthread_attr_init(&attr);
        pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
        param.sched_priority = sched_get_priority_min(SCHED_FIFO);

        pthread_attr_setschedparam(&attr, &param);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

        if (pthread_create(&idle, &attr, &idleThread, 0) != 0) {
               /* Creating the p-thread with the SCHED_FIFO policy failed.
               * Most probably this application has no superuser privileges,
               * so we just fall back to the default SCHED_OTHER policy
               * and priority 0.
               */
            pthread_attr_setschedpolicy(&attr, SCHED_OTHER);
            param.sched_priority = 0;
            pthread_attr_setschedparam(&attr, &param);
            Q_ALLEGE(pthread_create(&idle, &attr, &idleThread, 0) == 0);
        }
        pthread_attr_destroy(&attr);
    }

    return (uint8_t)1;
}
/*..........................................................................*/
void QS_onCleanup(void) {
    l_running = (uint8_t)0;
    QSPY_stop();
}
/*..........................................................................*/
void QS_onFlush(void) {
    uint16_t nBytes = 1024;
    uint8_t const *block;
    QF_CRIT_ENTRY(dummy);
    while ((block = QS_getBlock(&nBytes)) != (uint8_t *)0) {
        QF_CRIT_EXIT(dummy);
        QSPY_parse(block, nBytes);
        nBytes = 1024;
    }
}
/*..........................................................................*/
QSTimeCtr QS_onGetTime(void) {
    return (QSTimeCtr)clock();                                   // see NOTE01
}
/*..........................................................................*/
void QSPY_onPrintLn(void) {
    fputs(QSPY_line, stdout);
    fputc('\n', stdout);
}

/*****************************************************************************
* NOTE01:
* clock() is the most portable facility, but might not provide the desired
* granularity. Other, less-portable alternatives are clock_gettime(), rdtsc(),
* or gettimeofday().
*/

#endif                                                             /* Q_SPY */
/*--------------------------------------------------------------------------*/
