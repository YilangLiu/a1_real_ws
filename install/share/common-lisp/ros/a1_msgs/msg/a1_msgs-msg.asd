
(cl:in-package :asdf)

(defsystem "a1_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Contact" :depends-on ("_package_Contact"))
    (:file "_package_Contact" :depends-on ("_package"))
    (:file "FootPlanDiscrete" :depends-on ("_package_FootPlanDiscrete"))
    (:file "_package_FootPlanDiscrete" :depends-on ("_package"))
    (:file "FootState" :depends-on ("_package_FootState"))
    (:file "_package_FootState" :depends-on ("_package"))
    (:file "MultiFootPlanDiscrete" :depends-on ("_package_MultiFootPlanDiscrete"))
    (:file "_package_MultiFootPlanDiscrete" :depends-on ("_package"))
  ))