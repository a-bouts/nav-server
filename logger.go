package main

import "log"

type Logger struct {
	debug bool
}

func (l *Logger) infoln(args ...interface{}) {
	log.Println(args...)
}

func (l *Logger) infof(format string, args ...interface{}) {
	log.Printf(format, args...)
}

func (l *Logger) debugln(args ...interface{}) {
	if l.debug {
		log.Println(args...)
	}
}

func (l *Logger) debugf(format string, args ...interface{}) {
	if l.debug {
		log.Printf(format, args...)
	}
}
