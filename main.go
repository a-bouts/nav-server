package main

import (
	"flag"
	"net/http"
	"os"

	"github.com/peterbourgon/ff"
	log "github.com/sirupsen/logrus"

	"github.com/a-bouts/nav-server/api"
	"github.com/a-bouts/nav-server/land"
	"github.com/a-bouts/nav-server/wind"
	"github.com/a-bouts/nav-server/xmpp"

	_ "net/http/pprof"
)

func main() {

	fs := flag.NewFlagSet("start", flag.ExitOnError)
	var (
		debug      = fs.Bool("debug", false, "")
		cpuprofile = fs.Bool("cpuprofile", false, "")

		xmppHost     = fs.String("xmpp-host", "", "")
		xmppJid      = fs.String("xmpp-jid", "", "")
		xmppPassword = fs.String("xmpp-password", "", "")
		xmppTo       = fs.String("xmpp-to", "", "")
	)
	ff.Parse(fs, os.Args[1:], ff.WithEnvVarNoPrefix())

	log.SetOutput(os.Stdout)
	log.SetFormatter(&log.TextFormatter{
		FullTimestamp:          true,
		DisableLevelTruncation: true,
		PadLevelText:           true,
	})

	if *debug {
		log.SetLevel(log.DebugLevel)
	}

	x := &xmpp.Xmpp{Config: xmpp.Config{Host: *xmppHost, Jid: *xmppJid, Password: *xmppPassword, To: *xmppTo}}

	log.Info("Load lands")
	l, err := land.InitLand()
	if err != nil {
		log.Fatal("Error loading lands")
	}

	log.Info("Load winds")
	p := wind.InitWinds()

	router := api.InitServer(*cpuprofile, l, p, x)

	log.Println("Start server on port 8888")
	log.Fatal(http.ListenAndServe(":8888", router))
}
