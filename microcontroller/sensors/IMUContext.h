/* Class for abstracting IMU operations for simplewalker
 * TODO:
 *      sometimes read_and_send takes 500 ms (that's very slow) (PICO_STDIO_USB_STDOUT_TIMEOUT_US ?)
 *          happens twice in a row
 *          can trigger IMU_ERR_SEM if it happens while send_IMU_info() is about to request comm_sem
 *      sometimes both IMUData and IMUInfo take 2400, 3400 us, triggering IMU_ERR_SEM
 *      separate IMUState->start_read() and IMUState-send()?
 *          would allow us to spend less time in irq_callback(), if that's important
 */
#ifndef SIMPLEWALKER_PICO_IMUCONTEXT_H
#define SIMPLEWALKER_PICO_IMUCONTEXT_H

#include <memory>
#include "../../communication/communication.hpp"
#include "../../communication/messages.h"

using std::unique_ptr, std::make_unique, std::shared_ptr, std::make_shared;
struct IMUOverTempCheck;
typedef struct semaphore semaphore_t;
class MPU6050;

class IMUContext {
    unique_ptr<MessageOutbox<IMUDataMsg>> DataOutbox;
    unique_ptr<MessageOutbox<IMUInfoMsg>> SelfInfoOutbox;
    unique_ptr<MPU6050> IMU;
    unique_ptr<IMUOverTempCheck> overTempCheck;
    shared_ptr<semaphore_t> comm_sem;
    unique_ptr<semaphore_t> shared_data_sem, main_loop_active_sem;
    uint32_t irq_run_time;
    uint16_t read_and_send_err;
    float IMU_temp;
public:
    IMUContext(Communicator &comm, shared_ptr<semaphore_t> comm_semaphore);
    void setup_IMU();
    // read_and_send() will toggle the IMU sleep mode if it is over the temperature limit
    void read_and_send();
    void send_IMUdata_missing(bool ignore_sem=false);
    void send_IMU_info();
    ~IMUContext();
};

#endif //SIMPLEWALKER_PICO_IMUCONTEXT_H
