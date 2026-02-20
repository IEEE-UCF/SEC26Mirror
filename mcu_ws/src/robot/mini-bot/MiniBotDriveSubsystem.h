#include "src/robot/drive-base/RobotDriveBase.h"

namespace Subsystem {

class MiniBotDriveSusbsystemSetup : Classes::BaseSetup {

}

class MiniBotDriveSubsystem : public IMicroRosParticipant,
                              public Subsystem::TimedSubsystem {
public:
private:
};
} // namespace Subsystem