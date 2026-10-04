#include <gtest/gtest.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#include <tinyxml2.h>
#include "p4_common/calibration.hpp"
#include "p4_common/joint_order.hpp"
#include "p4_common/conversion.hpp"

namespace fs = std::filesystem;


void ExpectThrowNaming(const std::string & path,
                       const std::vector<std::string> & must_contain)
{
  try {
    p4_common::LoadCalibration(path);
    FAIL() << "expected LoadCalibration to throw, but it returned";
  } catch (const std::runtime_error & e) {
    const std::string msg = e.what();
    for (const auto & needle : must_contain) {
      EXPECT_NE(msg.find(needle), std::string::npos)
        << "message did not mention '" << needle << "'\n"
        << "actual message: " << msg;
    }
  }
}

class CalibrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    dir_ = fs::temp_directory_path() / "p4_common_calibration_test";
    fs::remove_all(dir_);      // wipe leftovers from a crashed run
    fs::create_directories(dir_);
  }

  void TearDown() override
  {
    fs::remove_all(dir_);
  }

  // Write text to <temp dir>/name and hand back the full path.
  std::string WriteFile(const std::string & name, const std::string & text)
  {
    const fs::path p = dir_ / name;
    std::ofstream out(p);
    out << text;
    out.close();
    return p.string();
  }

  fs::path dir_;
};

TEST_F(CalibrationTest, GoodFileLoads)
{
  const std::string yaml =
    "servos:\n"
    "  shoulder_pan:  {offset: 2048, servo_id: 1, sign:  1}\n"
    "  shoulder_lift: {offset: 1100, servo_id: 2, sign: -1}\n"
    "  elbow_flex:    {offset:  900, servo_id: 3, sign:  1}\n"
    "  wrist_flex:    {offset: 3000, servo_id: 4, sign:  1}\n"
    "  wrist_roll:    {offset:  512, servo_id: 5, sign: -1}\n"
    "  gripper:       {offset: 4095, servo_id: 6, sign:  1}\n";

  const std::string path = WriteFile("good.yaml", yaml);

  const auto cal = p4_common::LoadCalibration(path);

  const auto pan = static_cast<std::size_t>(p4_common::Joints::shoulder_pan);
  EXPECT_EQ(cal[pan].offset, 2048);
  EXPECT_EQ(cal[pan].servo_id, 1);
  EXPECT_EQ(cal[pan].sign, 1);

  const auto roll = static_cast<std::size_t>(p4_common::Joints::wrist_roll);
  EXPECT_EQ(cal[roll].offset, 512);
  EXPECT_EQ(cal[roll].servo_id, 5);
  EXPECT_EQ(cal[roll].sign, -1);
}

TEST_F(CalibrationTest, MissingJointThrows)
{
  const std::string yaml =
    "servos:\n"
    "  shoulder_pan:  {offset: 2048, servo_id: 1, sign:  1}\n"
    "  elbow_flex:    {offset:  900, servo_id: 3, sign:  1}\n"
    "  wrist_flex:    {offset: 3000, servo_id: 4, sign:  1}\n"
    "  wrist_roll:    {offset:  512, servo_id: 5, sign: -1}\n"
    "  gripper:       {offset: 4095, servo_id: 6, sign:  1}\n";

  const std::string path = WriteFile("missing_joint.yaml", yaml);
  const std::vector<std::string> must_contain = {"shoulder_lift"};
  ExpectThrowNaming(path, must_contain);
}
namespace
{

struct JointLimits
{
  double lower;
  double upper;
};

// Position of a joint name in the canonical order, or nothing.
std::optional<std::size_t> JointNameToIndex(const std::string & name)
{
  for (std::size_t i = 0; i < p4_common::kNumJoints; ++i) {
    if (name == p4_common::joint_order[i]) {
      return i;
    }
  }
  return std::nullopt;
}

// Read every revolute joint's limits out of the URDF, indexed by joint order.
// Throws if the file is unreadable or any of the six is missing -- an empty
// table would make the test below pass while checking nothing.
std::array<JointLimits, p4_common::kNumJoints> LoadUrdfLimits(const std::string & path)
{
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("could not read URDF: " + path);
  }

  const tinyxml2::XMLElement * robot = doc.FirstChildElement("robot");
  if (robot == nullptr) {
    throw std::runtime_error("URDF has no <robot> element: " + path);
  }

  std::array<JointLimits, p4_common::kNumJoints> limits{};
  std::array<bool, p4_common::kNumJoints> seen{};

  for (const tinyxml2::XMLElement * j = robot->FirstChildElement("joint");
       j != nullptr;
       j = j->NextSiblingElement("joint"))
  {
    // gripper_frame_joint is type="fixed" -- seven tags, six actuated.
    const char * type = j->Attribute("type");
    if (type == nullptr || std::string(type) != "revolute") {
      continue;
    }

    const char * name = j->Attribute("name");
    if (name == nullptr) {
      throw std::runtime_error("revolute joint with no name attribute");
    }

    const std::optional<std::size_t> index = JointNameToIndex(name);
    if (!index.has_value()) {
      throw std::runtime_error(
              "URDF revolute joint not in joint_order: " + std::string(name));
    }

    const tinyxml2::XMLElement * limit = j->FirstChildElement("limit");
    if (limit == nullptr) {
      throw std::runtime_error("joint has no <limit>: " + std::string(name));
    }

    // QueryDoubleAttribute reports a missing attribute. DoubleAttribute
    // would quietly hand back 0.0.
    double lower = 0.0;
    double upper = 0.0;
    if (limit->QueryDoubleAttribute("lower", &lower) != tinyxml2::XML_SUCCESS ||
      limit->QueryDoubleAttribute("upper", &upper) != tinyxml2::XML_SUCCESS)
    {
      throw std::runtime_error(
              "joint <limit> missing lower or upper: " + std::string(name));
    }

    limits[*index] = {lower, upper};
    seen[*index] = true;
  }

  for (std::size_t i = 0; i < p4_common::kNumJoints; ++i) {
    if (!seen[i]) {
      throw std::runtime_error(
              "URDF had no revolute joint named " +
              std::string(p4_common::joint_order[i]));
    }
  }

  return limits;
}

}  // namespace

// The real calibration must place every URDF joint limit inside the
// servo's 0-4095 tick range. An offset that looks fine at zero can
// still push a legal angle off the end of the range -- wrist_roll has
// only ~450 ticks of slack across its whole span.
TEST(RealCalibration, UrdfLimitsMapInsideTickRange)
{
  const auto cal = p4_common::LoadCalibration(CALIBRATION_PATH);
  const auto limits = LoadUrdfLimits(URDF_PATH);

  for (std::size_t i = 0; i < p4_common::kNumJoints; ++i) {
    const std::string name(p4_common::joint_order[i]);

    const double lo_ticks = p4_common::RadToTicks(limits[i].lower, cal[i]);
    const double hi_ticks = p4_common::RadToTicks(limits[i].upper, cal[i]);

    EXPECT_GE(lo_ticks, 0.0) << name << " lower limit maps to "
                             << lo_ticks << " ticks";
    EXPECT_LE(lo_ticks, 4095.0) << name << " lower limit maps to "
                                << lo_ticks << " ticks";
    EXPECT_GE(hi_ticks, 0.0) << name << " upper limit maps to "
                             << hi_ticks << " ticks";
    EXPECT_LE(hi_ticks, 4095.0) << name << " upper limit maps to "
                                << hi_ticks << " ticks";
  }
}