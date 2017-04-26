
(cl:in-package :asdf)

(defsystem "basic_exercises-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "resultbag" :depends-on ("_package_resultbag"))
    (:file "_package_resultbag" :depends-on ("_package"))
    (:file "pixelcountActionResult" :depends-on ("_package_pixelcountActionResult"))
    (:file "_package_pixelcountActionResult" :depends-on ("_package"))
    (:file "pixelcountActionGoal" :depends-on ("_package_pixelcountActionGoal"))
    (:file "_package_pixelcountActionGoal" :depends-on ("_package"))
    (:file "pixelcountFeedback" :depends-on ("_package_pixelcountFeedback"))
    (:file "_package_pixelcountFeedback" :depends-on ("_package"))
    (:file "pixelcountAction" :depends-on ("_package_pixelcountAction"))
    (:file "_package_pixelcountAction" :depends-on ("_package"))
    (:file "pixelcountActionFeedback" :depends-on ("_package_pixelcountActionFeedback"))
    (:file "_package_pixelcountActionFeedback" :depends-on ("_package"))
    (:file "pixelcountGoal" :depends-on ("_package_pixelcountGoal"))
    (:file "_package_pixelcountGoal" :depends-on ("_package"))
    (:file "pixelcountResult" :depends-on ("_package_pixelcountResult"))
    (:file "_package_pixelcountResult" :depends-on ("_package"))
  ))