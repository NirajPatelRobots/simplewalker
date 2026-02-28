
#include "IMUContext.h"
#include "pico/sem.h"
#include "IMU_errcheck.h"
#include "../micro_parameters.h"


const int DATA_SEM_WAIT_US = 200;


IMUContext::IMUContext(Communicator &comm, shared_ptr<semaphore_t> comm_semaphore)
    : DataOutbox(make_unique<MessageOutbox<IMUDataMsg>>(IMUDataMsgID, comm)),
      SelfInfoOutbox(make_unique<MessageOutbox<IMUInfoMsg>>(IMUInfoMsgID, comm)),
      IMU(make_unique<MPU6050>(DataOutbox->message.accel, DataOutbox->message.gyro)),
      overTempCheck(make_unique<IMUOverTempCheck>(IMU_TEMP_MAX)),
      comm_sem(std::move(comm_semaphore)),
      shared_data_sem(make_unique<semaphore_t>()),
      main_loop_active_sem(make_unique<semaphore_t>()),
      irq_run_time(),
      read_and_send_err(),
      IMU_temp()
    {
    SelfInfoOutbox->message.ID = IMUInfoMsgID;
    SelfInfoOutbox->message.debug_float = 0;
    SelfInfoOutbox->message.debug_int = 0;
    DataOutbox->message.ID = IMUDataMsgID;
    sem_init(shared_data_sem.get(), 1, 1);
    sem_init(main_loop_active_sem.get(), 1, 1);
}

void IMUContext::setup_IMU() {
    IMU->reset();
    IMU->power(1, false, false, false);
    IMU->set_timing(IMU_LP_FILTER_CFG_IDX, IMU_DT_MS - 1);
    IMU->configure_interrupt(false, false, true, true, true);
}

void IMUContext::read_and_send() {
    if (!sem_try_acquire(main_loop_active_sem.get())) {  // prevent double triggering
        if (sem_acquire_timeout_us(shared_data_sem.get(), DATA_SEM_WAIT_US)) {
            read_and_send_err |= IMU_ERR_OVERLOOP;
            sem_release(shared_data_sem.get());
        }
        return;
    }
    static const uint32_t DLPF_delay_us = (uint32_t)(IMU->read_timing().accel_timing.delay * 1e3);
    static const IMUScaleCheck scaleCheck(MPU6050::Scale_0, MPU6050::Scale_0);
    DataOutbox->message.timestamp_us = (uint32_t)to_us_since_boot(get_absolute_time()) - DLPF_delay_us;
    IMU->read();
    DataOutbox->message.errcode = overTempCheck->check_temp_err(*IMU);
    scaleCheck.set_msg(DataOutbox->message);
    if (sem_acquire_timeout_us(comm_sem.get(), COMM_SEM_WAIT_US)) {
        DataOutbox->send();
        sem_release(comm_sem.get());
    } else {
        if (sem_acquire_timeout_us(shared_data_sem.get(), DATA_SEM_WAIT_US)) {
            read_and_send_err |= IMU_ERR_SEM;
            sem_release(shared_data_sem.get());
        }
    }
    uint32_t new_run_time =
            (uint32_t) to_us_since_boot(get_absolute_time()) - DLPF_delay_us - DataOutbox->message.timestamp_us;
    if (sem_acquire_timeout_us(shared_data_sem.get(), DATA_SEM_WAIT_US)) {
        read_and_send_err |= DataOutbox->message.errcode;
        IMU_temp = IMU->chip_temp;
        if (new_run_time > irq_run_time)
            irq_run_time = new_run_time;
        sem_release(shared_data_sem.get());
    }
    sem_release(main_loop_active_sem.get());
}

void IMUContext::send_IMUdata_missing(bool ignore_sem) {
    DataOutbox->message.errcode = IMU_ERR_NO_IRQ + overTempCheck->check_temp_err(*IMU)
                                     + (IMU_ERR_NOT_CONNECTED * IMU->is_connected());
    if (ignore_sem || sem_acquire_timeout_us(shared_data_sem.get(), DATA_SEM_WAIT_US)) {
        read_and_send_err |= DataOutbox->message.errcode;
        IMU_temp = IMU->chip_temp;
        if (!ignore_sem) sem_release(shared_data_sem.get());
    }
    if (ignore_sem || sem_acquire_timeout_us(comm_sem.get(), COMM_SEM_WAIT_US)) {
        DataOutbox->send();
        if (!ignore_sem) sem_release(comm_sem.get());
    } else {
        SelfInfoOutbox->message.errcode |= IMU_ERR_SEM;
    }
}

void IMUContext::send_IMU_info() {
    SelfInfoOutbox->message.timestamp_us = (uint32_t)to_us_since_boot(get_absolute_time());
    if (sem_acquire_timeout_us(shared_data_sem.get(), DATA_SEM_WAIT_US)) {
        SelfInfoOutbox->message.IMU_temp = IMU_temp;
        SelfInfoOutbox->message.run_time_us = irq_run_time;
        irq_run_time = 0;
        SelfInfoOutbox->message.errcode |= read_and_send_err;
        read_and_send_err = 0;
        sem_release(shared_data_sem.get());
    } else {
        SelfInfoOutbox->message.errcode |= IMU_ERR_INFO_SEM;
    }
    if (sem_acquire_timeout_us(comm_sem.get(), COMM_SEM_WAIT_US)) {
        SelfInfoOutbox->send();
        sem_release(comm_sem.get());
        SelfInfoOutbox->message.errcode = 0;
    } else {
        SelfInfoOutbox->message.errcode |= IMU_ERR_INFO_SEM;
    }
    SelfInfoOutbox->message.info_run_time_us = (uint32_t)to_us_since_boot(get_absolute_time()) - SelfInfoOutbox->message.timestamp_us;
}

IMUContext::~IMUContext() = default;
