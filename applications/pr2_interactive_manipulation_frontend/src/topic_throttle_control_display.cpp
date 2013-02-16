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

/* Just initilialize everything and connect the buttons for the display 
 * to their callbacks.
 */
TopicThrottleControlDisplay::TopicThrottleControlDisplay(): 
  Display(),
  //frame_(NULL),
  //frame_dock_(NULL),  
  set_rate_ac_(NULL),
  send_msg_ac_(NULL)

{
  topic_rate_ = new rviz::FloatProperty("Max Frequency",0.0,
                                        "Maximum number of messages/second."
                                        "0.0 stops messages",
                                        this,
                                        SLOT(updateTopicFrequency() ));


 topic_name_ = new rviz::EditableEnumProperty("Throttled Topic","",
                                              "Throttled Topic to control",
                                              this,
                                              SLOT(updateTopicName() ));

 snapshot_ = new rviz::BoolProperty("Snapshot","",
                                    "Request a single message from this topic,"
                                    "even if it is currently paused",
                                    this,
                                    SLOT(getOneMessage()));
 

 connect(topic_name_,SIGNAL(requestOptions(EditableEnumProperty*)),
         this, SLOT(fillTopicList()));
}
  



TopicThrottleControlDisplay::~TopicThrottleControlDisplay()
{
  // Delete the UI element if it exists   
  // delete frame_dock_;    
  
  // Delete action clients
  delete set_rate_ac_;
  delete send_msg_ac_;
}



void TopicThrottleControlDisplay::onInitialize()
{
  rviz::WindowManagerInterface* window_context = context_->getWindowManager();
  ROS_ASSERT(window_context);
  // FIXME add GUI
  //frame_ = new TopicThrottleControl(context_, 
  //                                  window_context->getParentWindow());
  //frame_dock_ = window_context->addPane( "TopicThrottle", frame_ );

}



void TopicThrottleControlDisplay::onEnable()
{
  // Do GUI stuff
  //frame_dock_->show();
}



void TopicThrottleControlDisplay::onDisable()
{
  // Do GUI stuff
  //if ( frame_dock_ ) frame_dock_->hide();
}



void TopicThrottleControlDisplay::update(float, float)
{
  // DO gui stuff
  // if ( frame_ ) frame_->update();
}



/* Helper function to determine if a topic is dynamically reconfigurable 
 * by testing whether or not it has topics associated with the rate 
 * throttling action.
 */
bool isGrandparentThrottleable(const ros::master::TopicInfo & topic)
{
  //Define the correct message type as the message type of interest
  typedef ros::message_traits::
    DataType<advanced_nodelet_throttle::SetRateActionFeedback> MsgTypeStr;
    
  //Store the string that names the message type
  static const std::string topic_type = std::string(MsgTypeStr().value());
  
  return topic.datatype==topic_type;
}



// Helper function to get name of the node that is dynamically reconfigurable
std::string getGrandparentName(std::string topic_name)
{
  QString name(topic_name.c_str());
  name.truncate(name.lastIndexOf('/'));
  name.truncate(name.lastIndexOf('/'));
  return name.toStdString();
}




void TopicThrottleControlDisplay::setTopicRate(std::string topic_name, float rate)
{
  // Wait for valid action server
  if(!set_rate_ac_->waitForServer(ros::Duration(5.0)))
  {
    setStatus(rviz::StatusProperty::Error, 
              "Couldn't reach action server", "Error");
  }
  else
  {
    advanced_nodelet_throttle::SetRateGoal g;
    g.rate = rate;
    set_rate_ac_->sendGoal(g, 
                           boost::bind(&TopicThrottleControlDisplay::
                                       setTopicRateDoneCallback, this, _1, _2)
                           );
  }
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
   if( isGrandparentThrottleable(topic))
   {
     std::string parent_topic_name = getGrandparentName(topic.name);
     topic_name_->addOptionStd(parent_topic_name);
   }
 }
 topic_name_->sortOptions();
 QApplication::restoreOverrideCursor();
}



bool TopicThrottleControlDisplay::getRate()
{
  double rate;
  bool ok = ros::param::get(topic_name_->getStdString()
                            + "/data_throttle/update_rate",
                            rate);
  
  if (!ok)
    return false;
  topic_rate_->setValue(rate);
  return true;
}




void TopicThrottleControlDisplay::updateTopicFrequency()
{
  setTopicRate(topic_name_->getStdString(), topic_rate_->getFloat());           
}




/// When the topic name is updated, go ahead and find out the
/// current publication rate
void TopicThrottleControlDisplay::updateTopicName()
{
  // delete existing action clients
  delete set_rate_ac_;
  delete send_msg_ac_;

  // create action clients
  set_rate_ac_ = new actionlib::SimpleActionClient<advanced_nodelet_throttle::
                                SetRateAction>(topic_name_->getStdString() 
                                               + "/set_rate", 
                                               true);


  send_msg_ac_ = new actionlib::SimpleActionClient<advanced_nodelet_throttle::
                                SendOneMsgAction>(topic_name_->getStdString() 
                                                  + "/request_single_msg", 
                                                  true);
 
  // update rate
  if (!getRate())
    setStatus(rviz::StatusProperty::Error, 
              "Topic does not have reconfigurable parameter required",
              "Error");
 
  return;
}




  
void TopicThrottleControlDisplay::setTopicRateDoneCallback(
        const actionlib::SimpleClientGoalState &state, 
        const advanced_nodelet_throttle::SetRateResultConstPtr &result)
{

  if(state != actionlib::SimpleClientGoalState::SUCCEEDED)
    setStatus(rviz::StatusProperty::Error, "Set Topic Rate Failed", "Error");
  else
    setStatus(rviz::StatusProperty::Ok, "Set Topic Rate Succeede", "Ok");

}





 void TopicThrottleControlDisplay::sendMsgDoneCallback(
         const actionlib::SimpleClientGoalState &state, 
         const advanced_nodelet_throttle::SendOneMsgResultConstPtr &result)
{
  if(state != actionlib::SimpleClientGoalState::SUCCEEDED)
    setStatus(rviz::StatusProperty::Error, 
              "Single Message Request Failed", 
              "Error");
}



void TopicThrottleControlDisplay::getOneMessage()
{
  if (!send_msg_ac_)
    return;
  if(!send_msg_ac_->waitForServer(ros::Duration(5.0)))
    {
      setStatus(rviz::StatusProperty::Error, 
                "Couldn't reach action server", 
                "Error");
    }
  else
  {
    advanced_nodelet_throttle::SendOneMsgGoal g;

    send_msg_ac_->sendGoal(g, boost::bind(&TopicThrottleControlDisplay
                                          ::sendMsgDoneCallback,
                                          this,
                                          _1, _2));    
  }
}


}//namespace
