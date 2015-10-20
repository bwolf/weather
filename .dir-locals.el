;;; Directory Local Variables
;;; For more information see (info "(emacs) Directory Variables")

((nil . ((indent-tabs-mode . nil)))
 (c-mode . ((eval . (progn
                      (semantic-mode -1)
                      (message "Disabled semantic-mode via .dir-locals.el")
                      (setq flycheck-checker 'c/c++-gcc
                            flycheck-c/c++-gcc-executable "/usr/local/bin/avr-gcc"
                            flycheck-gcc-args '("-mmcu=atmega88pa" "-Os")
                            flycheck-gcc-definitions '("F_OSC=1000000")
                            flycheck-gcc-language-standard "gnu99"
                            flycheck-gcc-warnings '("all" "extra" "strict-prototypes" "fatal-errors"))
                      (set (make-local-variable 'company-c-headers-path-system) nil)
                      (setq company-c-headers-path-system
                            '("/usr/local/Cellar/avr-libc/1.8.1/avr/include"))
                      (setq comment-start "//"
                            comment-end ""
                            coment-continue "//")
                      )))))
