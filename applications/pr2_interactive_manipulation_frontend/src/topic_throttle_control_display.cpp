/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include "pr2_interactive_manipulation/topic_throttle_control_display.h"

#include <rviz/display_context.h>
#include <rviz/panel_dock_widget.h>
#include <rviz/window_manager_interface.h>
#include <rviz/properties/status_property.h>
#include <QThread>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <QDockWidget>
#include <QApplication>
#include <QStringList>

namespace pr2_interactive_manipulation {

/* Just initilialize everything and connect the buttons for the display to their callbacks.
 */
TopicThrottleControlDisplay::TopicThrottleControlDisplay(): 
  Display(),
  frame_(0),
  frame_dock_(0),
  set_rate_thread_(NULL)
{
  topic_rate_ = new rviz::FloatProperty("Max Frequency",0.0,
                      "Maximum number of messages/second. 0.0 stops messages",
                      this,
                      SLOT(updateTopicFrequency() ));


 topic_name_ = new rviz::EditableEnumProperty("Throttled Topic","",
                      "Throttled Topic to control",
                      this,
                      SLOT(updateTopicName() ));

 connect(topic_name_,SIGNAL(requestOptions(EditableEnumProperty*)),this, SLOT(fillTopicList()));
}
  



TopicThrottleControlDisplay::~TopicThrottleControlDisplay()
{
  // Delete the UI element if it exists
  // Any existing frame dock will delete frames that it holds, so don't delete them explicitly
  if(frame_dock_)
    delete frame_dock_;    
  
  // If a thread has to call dynamic_reconfigure asynchronously has been allocated, 
  // wait for it to finish, and then delete it so that we don't end up leaking it
  // or segfaulting when it reaches its exit function
  if(set_rate_thread_)
  {
    set_rate_thread_->wait();
    delete set_rate_thread_;
  }
}

void TopicThrottleControlDisplay::onInitialize()
{
  rviz::WindowManagerInterface* window_context = context_->getWindowManager();
  ROS_ASSERT(window_context);
  // FIXME add GUI
  frame_ = new QFrame();//TopicThrottleControl(context_, window_context->getParentWindow());
  frame_dock_ = window_context->addPane( "TopicThrottle", frame_ );

}

void TopicThrottleControlDisplay::onEnable()
{
  frame_dock_->show();
}

void TopicThrottleControlDisplay::onDisable()
{
  if ( frame_dock_ ) frame_dock_->hide();
}

void TopicThrottleControlDisplay::update(float, float)
{
  if ( frame_ ) frame_->update();
}

// Helper function to determine if a topic is dynamically reconfigurable by testing whether or not it 
// has topics associated with the dynamic_reconfigure system associated with it.
bool isParentReconfigurable(const ros::master::TopicInfo & topic)
{
  static const std::string topic_config_type(QString::fromStdString(
                                             ros::message_traits::datatype<dynamic_reconfigure::ConfigDescription>()).toStdString());
  
  return topic.datatype==topic_config_type;
}

// Helper function to get name of the node that is dynamically reconfigurable
std::string getParentName(std::string topic_name)
{
  QString name(topic_name.c_str());
  name.truncate(name.lastIndexOf('/'));
  return name.toStdString();
}

void TopicThrottleControlDisplay::setTopicRate(std::string topic_name, float rate)
{
  // If a thread already exists, wait for it to finish and get rid of it
  if(set_rate_thread_)
  {
    set_rate_thread_->wait();
    delete set_rate_thread_;    
  }
  // Create and dispatch the thread
  set_rate_thread_ = new DynRecThread(topic_name, rate);
  // Connect the signal to collect the thread when it has finished
  connect(set_rate_thread_, SIGNAL(finishedRunning(int)) , this, SLOT(finishedSetting(int)));
  // Dispatch the thread
  set_rate_thread_->start();  
}


void TopicThrottleControlDisplay::finishedSetting(int succeeded)
{
  if (!succeeded)
    setStatus(rviz::StatusProperty::Error, "Rate could not be set!", "Error");
    
}

// Helper function to test whether the parent has an update rate parameter
// FIXME: Ideally this would test if the update_rate parameter is actually
// dynamically reconfigurable, but doing so is extremely slow and hackish
// with the current (nonexistent) c++ dynamic_reconfigure interface.
bool isRateReconfigurable(std::string topic_name)
{
  if (ros::param::has(topic_name + "/update_rate"))
  {    
    return true;
  }
  return false;
}




void TopicThrottleControlDisplay::fillTopicList()
{
  QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
  topic_name_->clearOptions();
  ros::master::V_TopicInfo topics;
  ros::master::getTopics( topics );
  // Loop through all published topics
  ros::master::V_TopicInfo::iterator it;
  for( it = topics.begin(); it != topics.end(); ++it )
  {
   const ros::master::TopicInfo& topic = *it;
   // Only add topics with a reconfigurable rate;
   if( isParentReconfigurable(topic))
   {
     std::string parent_topic_name = getParentName(topic.name);
     topic_name_->addOptionStd(parent_topic_name);
   }
 }
 topic_name_->sortOptions();
 QApplication::restoreOverrideCursor();
}



bool TopicThrottleControlDisplay::getRate()
{
  double rate;
  bool ok = ros::param::get(topic_name_->getStdString()+"/update_rate", rate);
  if (!ok)
    return false;
  topic_rate_->setValue(rate);
  return true;
}

void TopicThrottleControlDisplay::updateTopicFrequency()
{
  setTopicRate(topic_name_->getStdString(), topic_rate_->getFloat());           
}

// When the topic name is updated, go ahead and find out the current publication rate
void TopicThrottleControlDisplay::updateTopicName()
{
  if (!getRate())
    setStatus(rviz::StatusProperty::Error, "Topic does not have reconfigurable parameter required", "Error");
}


void DynRecThread::run()
{
  std::string command_str = "rosrun dynamic_reconfigure dynparam set "
    + topic_name_
    + " update_rate " + QString::number(rate_).toStdString();
  Q_EMIT finishedRunning(system(command_str.c_str()) == 0);  
}



}
