;; Auto-generated. Do not edit!


(when (boundp 'mujoco_ros_msgs::SensorBase)
  (if (not (find-package "MUJOCO_ROS_MSGS"))
    (make-package "MUJOCO_ROS_MSGS"))
  (shadow 'SensorBase (find-package "MUJOCO_ROS_MSGS")))
(unless (find-package "MUJOCO_ROS_MSGS::SENSORBASE")
  (make-package "MUJOCO_ROS_MSGS::SENSORBASE"))

(in-package "ROS")
;;//! \htmlinclude SensorBase.msg.html


(defclass mujoco_ros_msgs::SensorBase
  :super ros::object
  :slots (_name _data ))

(defmethod mujoco_ros_msgs::SensorBase
  (:init
   (&key
    ((:name __name) "")
    ((:data __data) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _data __data)
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; float64[] _data
    (* 8    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; float64[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (sys::poke (elt _data i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get mujoco_ros_msgs::SensorBase :md5sum-) "de229af6c922862f640c59218b71e019")
(setf (get mujoco_ros_msgs::SensorBase :datatype-) "mujoco_ros_msgs/SensorBase")
(setf (get mujoco_ros_msgs::SensorBase :definition-)
      "string name

float64[] data

")



(provide :mujoco_ros_msgs/SensorBase "de229af6c922862f640c59218b71e019")


