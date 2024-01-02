;; Auto-generated. Do not edit!


(when (boundp 'a1_msgs::Contact)
  (if (not (find-package "A1_MSGS"))
    (make-package "A1_MSGS"))
  (shadow 'Contact (find-package "A1_MSGS")))
(unless (find-package "A1_MSGS::CONTACT")
  (make-package "A1_MSGS::CONTACT"))

(in-package "ROS")
;;//! \htmlinclude Contact.msg.html


(defclass a1_msgs::Contact
  :super ros::object
  :slots (_type _contacts ))

(defmethod a1_msgs::Contact
  (:init
   (&key
    ((:type __type) 0)
    ((:contacts __contacts) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _type (round __type))
   (setq _contacts __contacts)
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:contacts
   (&optional __contacts)
   (if __contacts (setq _contacts __contacts)) _contacts)
  (:serialization-length
   ()
   (+
    ;; int8 _type
    1
    ;; float64[] _contacts
    (* 8    (length _contacts)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _type
       (write-byte _type s)
     ;; float64[] _contacts
     (write-long (length _contacts) s)
     (dotimes (i (length _contacts))
       (sys::poke (elt _contacts i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _type
     (setq _type (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _type 127) (setq _type (- _type 256)))
   ;; float64[] _contacts
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _contacts (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _contacts i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get a1_msgs::Contact :md5sum-) "cdf685a63f9cb6c50837ec4316d75b8d")
(setf (get a1_msgs::Contact :datatype-) "a1_msgs/Contact")
(setf (get a1_msgs::Contact :definition-)
      "# 0 is position control, 1 is velocity control
int8 type
float64[] contacts

")



(provide :a1_msgs/Contact "cdf685a63f9cb6c50837ec4316d75b8d")


