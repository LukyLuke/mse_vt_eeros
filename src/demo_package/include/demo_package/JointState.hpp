#pragma once

#include <eeros/math/Matrix.hpp>

#include<string>
#include<vector>
#include <algorithm>


namespace eeros {
namespace control {

  // TODO: Don't use a matrix but the real type <JointState>.
  //       Signal and Block have _clear<T>() functions only for primitives,
  //       therefore complex types cannot be used.
  typedef eeros::math::Matrix<7, 1, double> JointStateSignal;

  struct JointStateData {
    JointStateData(std::string name, double position, double velocity, double effort) :
      name(name), position(position), velocity(velocity), effort(effort) {};

    // The name of this joint in the URDF-Model
    std::string name;

    // In Radians for revolute/continuous, meters for prismatic
    double position = 0.0;

    // Rate of change in position in "radians per second (r/s)" or "meters per second (m/s)"
    double velocity = 0.0;

    // Torque in "newton meters" for revolute/continuous or "newtons" for prismatic
    double effort = 0.0;
  };

  class JointState {
    public:
      JointState() {
        data = std::vector<std::shared_ptr<JointStateData>>();
      }

      JointState(const JointState& other) {
        this->data = std::vector<std::shared_ptr<JointStateData>>();
        for (size_t pos = 0; pos < other.data.size(); pos++) {
          this->data.push_back(other.data.at(pos));
        }
      }

      /**
       * Add a joint state
       *
       * @param name - Name of the joint
       * @param position - Position of the joint. Depending of the joint this can be an angle or length
       * @param velocity - Velocity this joint has
       * @param effort - Effort (energy?) used
       */
      void add(const std::string name, const double position = 0.0, const double velocity = 0.0, const double effort = 0.0) {
        data.push_back(std::make_shared<JointStateData>(name, position, velocity, effort));
      }

      /**
       * Get a reference to the JointStateData
       *
       * @return Vector of SharedPointer with all the JointStateData
       */
      std::vector<std::shared_ptr<JointStateData>> getJointStates() {
        return data;
      }

      /**
       * Creates JointStateSignal to work with EEROS-Signals
       *
       * @return JointStateSignal with only the positions of all JointStates
       */
      JointStateSignal getSignalStates() {
        JointStateSignal result;
        result.zero();
        auto positions = result.getColVector(0);
        for (size_t i = 0; i < std::min(positions.size(), data.size()); i++) {
          positions[i] = data.at(i)->position;
        }
        result.setCol(0, positions);
        return result;
      }

      /**
       * Find a joint by it's name
       *
       * @param name - The name of the joint to find
       * @return Shared Pointer with JointData or nullptr
       */
      std::shared_ptr<JointStateData> findJoint(const std::string name) {
        for (size_t pos = 0; pos < data.size(); pos++) {
          if ((data.at(pos) != nullptr) && (data.at(pos)->name == name)) {
            return data.at(pos);
          }
        }
        return nullptr;
      }

  private:
    std::vector<std::shared_ptr<JointStateData>> data;
  };

}
}
