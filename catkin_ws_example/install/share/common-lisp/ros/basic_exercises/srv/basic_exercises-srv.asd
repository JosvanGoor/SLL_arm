
(cl:in-package :asdf)

(defsystem "basic_exercises-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "rgb2grey" :depends-on ("_package_rgb2grey"))
    (:file "_package_rgb2grey" :depends-on ("_package"))
  ))