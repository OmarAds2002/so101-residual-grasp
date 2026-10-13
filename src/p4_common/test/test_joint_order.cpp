#include <gtest/gtest.h>
#include <tinyxml2.h>

#include <fstream>
#include <optional>
#include <set>
#include <sstream>
#include <string>

#include "p4_common/joint_order.hpp"

// Pull the names of every revolute joint out of a URDF string.
// nullopt means the XML did not parse, or there was no <robot> tag.
std::optional<std::set<std::string>> RevoluteJointNames(const std::string & xml)
{
  tinyxml2::XMLDocument doc;
  if (doc.Parse(xml.c_str()) != tinyxml2::XML_SUCCESS) {
    return std::nullopt;
  }

  const tinyxml2::XMLElement * robot = doc.FirstChildElement("robot");
  if (robot == nullptr) {
    return std::nullopt;
  }

  std::set<std::string> names;
  for (const tinyxml2::XMLElement * j = robot->FirstChildElement("joint"); j != nullptr;
       j = j->NextSiblingElement("joint")) {
    const char * type = j->Attribute("type");
    const char * name = j->Attribute("name");
    if (type == nullptr || name == nullptr) {
      continue;
    }
    if (std::string(type) == "revolute") {
      names.insert(name);
    }
  }
  return names;
}

std::string ReadFile(const std::string & path)
{
  std::ifstream in(path);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

TEST(JointOrder, NamesMatchUrdf)
{
  const std::string xml = ReadFile(URDF_PATH);
  ASSERT_FALSE(xml.empty()) << "could not read URDF at " << URDF_PATH;

  const auto from_urdf = RevoluteJointNames(xml);
  ASSERT_TRUE(from_urdf.has_value()) << "URDF did not parse";

  std::set<std::string> from_header;
  for (const auto & n : p4_common::joint_order) {
    from_header.insert(std::string(n));
  }

  EXPECT_EQ(*from_urdf, from_header);
}

TEST(JointOrder, DetectsMissingJoint)
{
  const std::string xml =
    "<robot name=\"x\">"
    "  <joint name=\"shoulder_pan\" type=\"revolute\"/>"
    "</robot>";

  const auto from_urdf = RevoluteJointNames(xml);
  ASSERT_TRUE(from_urdf.has_value());
  EXPECT_EQ(from_urdf->size(), 1u);
}
