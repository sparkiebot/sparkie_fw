#include "../uros/URosComponent.hpp"
#include <stdint.h>
#include <nav_msgs/msg/odometry.h>

namespace sparkie
{
    /**
     * @brief Component that handles odometry calculations and publishes odometry messages. <br>
     */
    class OdometryComponent : public URosComponent
    {
    public:
        OdometryComponent();
        virtual uint8_t getHandlesNum();

    protected:
        void rosInit();

    private:
        virtual void init();
        virtual void loop(TickType_t *xLastWakeTime);

        double x, y;
        double theta;
        double linear_velocity, angular_velocity;
        double raw_motor_data[2];

        nav_msgs__msg__Odometry odom_msg;
        
    };    
} // namespace sparkie
