package xmpp

import (
	"crypto/tls"
	"errors"
	"log"
	"strings"

	"github.com/mattn/go-xmpp"
)

type (
	// Config for the plugin.
	Config struct {
		Host     string
		Jid      string
		Password string
		To       string
	}

	Xmpp struct {
		Config Config
	}
)

func serverName(jid string) string {
	return strings.Split(jid, "@")[1]
}

func (x Xmpp) Send(message string) error {

	if len(x.Config.Jid) == 0 || len(x.Config.Password) == 0 || len(x.Config.To) == 0 {
		log.Println("missing xmpp config")

		return errors.New("missing xmpp config")
	}

	if len(x.Config.Host) == 0 {
		x.Config.Host = serverName(x.Config.Jid)
	}

	xmpp.DefaultConfig = tls.Config{
		InsecureSkipVerify: true,
	}

	options := xmpp.Options{
		Host:          x.Config.Host,
		User:          x.Config.Jid,
		Password:      x.Config.Password,
		NoTLS:         true,
		StartTLS:      true,
		Debug:         false,
		Session:       false,
		Status:        "xa",
		StatusMessage: "I for one welcome our new codebot overlords.",
	}

	log.Println("create client")
	log.Println(options)
	talk, err := options.NewClient()

	if err != nil {
		log.Println(err.Error())

		return err
	}

	// send message.
	log.Println("send message")
	talk.Send(xmpp.Chat{Remote: x.Config.To, Type: "chat", Text: message})

	return nil
}
