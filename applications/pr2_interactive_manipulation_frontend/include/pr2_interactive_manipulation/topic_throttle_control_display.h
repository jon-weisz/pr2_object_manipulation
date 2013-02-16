/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TOPIC_THROTTLE_CONTROL_DISPLAY_H
#define TOPIC_THROTTLE_CONTROL_DISPLAY_H


#include "rviz/display.h"
#include "rviz/properties/editable_enum_property.h"
#include "rviz/properties/float_property.h"
#include <QFrame>
#include <QThread>
#include <advanced_nodelet_throttle/SetRateAction.h>
#include <advanced_nodelet_throttle/SendOneMsgAction.h>
#include <actionlib/client/simple_action_client.h>


class QDockWidget;
class QStringList;


namespace rviz {
class DisplayContext;
class WindowManagerInterface;
class PanelDockWidget;
}

namespace pr2_interactive_manipulation
{


/* @brief Display to allow the user to reset the rate at which throttled 
 * nodes are published 
 */
class TopicThrottleControlDisplay : public rviz::Display
{
Q_OBJECT

public:
  TopicThrottleControlDisplay();
  
  virtual ~TopicThrottleControlDisplay();

  virtual void update(float wall_dt, float ros_dt);

  virtual void onInitialize();

protected Q_SLOTS:
  /* @brief Callback function to set the frequency of the throttled topic 
   * when the button is changed.
   */
  void updateTopicFrequency();

 /* @brief Callback function to set the name of the throttled topic 
  * when the button is changed.
  */
  void updateTopicName();

  /* @brief Callback function to fill the dropdown list of valid
   * throttle-able topics when the topic name drop down box is activated
   */
  void fillTopicList();

  /// @brief Request most recent message recieved by topic   
  void getOneMessage();

protected:
  /* @brief Callback to enable any rendered UI to the user 
   * when this display is enabled.    
   */
  virtual void onEnable();

  /* @brief Callback to remove any rendered UI to the user 
   * when this display is enabled. 
   */
  virtual void onDisable();

  /* @brief Queries the parameter server for the current rate of the throttled topic
   * and stores this value in topic_rate_ if successfull.
   *
   * Returns true iff the throttled topic has an update_rate parameter.
   */
  bool getRate();

  /// The rate at which the throttled topic is published. 
  rviz::FloatProperty *topic_rate_;
  /// The name of the topic being throttled
  rviz::EditableEnumProperty *topic_name_;
  /// Button to take a snapshot
  rviz::BoolProperty * snapshot_;
  

  //TODO Add some kind of GUI
  //TopicThrottleControl* frame_;  
  //QDockWidget* frame_dock_;

  /// @brief Set the publication frequency of the throttled topic
  void setTopicRate(std::string topic_name, float rate);
  /* @brief Callback to set status of display according to the status 
   * of the set topic rate action.
   */
  void setTopicRateDoneCallback(const actionlib::SimpleClientGoalState &state,
                                const advanced_nodelet_throttle::
                                SetRateResultConstPtr &result);

  /* @brief Callback to set status of display according to the status 
   * of the request message action.
   */
  void sendMsgDoneCallback(const actionlib::SimpleClientGoalState &state, 
                           const advanced_nodelet_throttle::
                           SendOneMsgResultConstPtr &result);
  
  
  typedef actionlib::SimpleActionClient<advanced_nodelet_throttle::
                                        SetRateAction> SetRateActionClient;
  typedef actionlib::SimpleActionClient<advanced_nodelet_throttle::
                                        SendOneMsgAction> SendMsgActionClient;

  SetRateActionClient *set_rate_ac_;
  SendMsgActionClient *send_msg_ac_;
};

}
#endif
