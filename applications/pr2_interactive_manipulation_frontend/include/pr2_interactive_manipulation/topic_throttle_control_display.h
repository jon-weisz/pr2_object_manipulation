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



class QDockWidget;
class QStringList;


namespace rviz {
class DisplayContext;
class WindowManagerInterface;
class PanelDockWidget;
}

namespace pr2_interactive_manipulation
{

/* @brief - Thread to dispatch calls to the command line dynamic_reconfigure interface asynchronously
 * because they are so very slow.
 */
class DynRecThread : public QThread
{
  Q_OBJECT
public:
  /* @brief - Creates a thread that runs the dynamic_reconfigure command to set the 
   *  rate of this topic.
   * 
   * @param topic_name - The name of the topic being reconfigured
   * @param rate - The new frequency to publish at. Any rate <=0.0 means don't publish.
   */  
  DynRecThread(std::string topic_name, float rate): topic_name_(topic_name), rate_(rate){}


  /* @brief - Does the actual work of dispatching the call to dynamic_reconfigure
   */
  virtual void run();


Q_SIGNALS:
  /* @brief - signal emitted to let the thread that originally dispatched this one
   *  know that we have finished.
   */
  void finishedRunning(int i);


private:
  std::string topic_name_;
  float rate_;
};


/* @brief - Display to allow the user to reset the rate at which throttled nodes are published
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

  /* @brief Callback function for when the asynchronous thread that actually
   * calls the dynamic reconfigure code has finished.
   *
   * @param succeeded - Whether the dispatched thread exited without error 
   * after setting the requested parameter
   */
  void finishedSetting(int succeeded);
  void getOneMessage();

protected:
  // @brief Callback to enable any rendered UI to the user when this display is enabled. 
   
  virtual void onEnable();

  // @brief Callback to remove any rendered UI to the user when this display is enabled. 
  virtual void onDisable();

  /* @brief Queries the parameter server for the current rate of the throttled topic
   * and stores this value in topic_rate_ if successfull.
   *
   * Returns true iff the throttled topic has an update_rate parameter.
   */
  bool getRate();
  // The rate at which the throttled topic is published. 
  rviz::FloatProperty *topic_rate_;
  // The name of the topic being throttled
  rviz::EditableEnumProperty *topic_name_;

  //FIXME design buttons for this sucker
  //TopicThrottleControl* frame_;
  QFrame * frame_;
  QDockWidget* frame_dock_;

  /* @brief Set the publication frequency of the throttled topic
   *
   * This function allocates and dispatches a thread to call the dynamic_reconfigure script
   * from the command line.
   */
  void setTopicRate(std::string topic_name, float rate);

  // Any currently existing thread for calling the dynamic_reconfigure script.
  DynRecThread * set_rate_thread_;
};

}
#endif
