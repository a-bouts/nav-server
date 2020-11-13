package main

import (
	"encoding/json"
	"flag"
	"log"
	"net/http"
	"os"
	"sync"
	"time"

	"github.com/gorilla/mux"
	"github.com/jasonlvhit/gocron"
	"github.com/peterbourgon/ff"

	"github.com/a-bouts/nav-server/wind"
	"github.com/a-bouts/nav-server/xmpp"

	_ "net/http/pprof"
)

type GoNav struct {
	Expes       map[string]bool `json:"expes"`
	Start       LatLon          `json:"start"`
	Bearing     int             `json:"bearing"`
	CurrentSail byte            `json:"currentSail"`
	Race        Race            `json:"race"`
	Delta       float64         `json:"delta"`
	MaxDuration float64         `json:"maxDuration"`
	Delay       int             `json:"delay"`
	Sail        int             `json:"sail"`
	Foil        bool            `json:"foil"`
	Hull        bool            `json:"hull"`
	Winch       bool            `json:"winch"`
	Malus       float64         `json:"malus"`
	Stop        bool            `json:"stop"`
}

func Expes(w http.ResponseWriter, req *http.Request) {

	expes := []string{
		"new-polars",
		"progressive-intervales",
		"sqrt-dist-from",
		"optim",
		"max-dist"}

	json.NewEncoder(w).Encode(expes)
}

func Refresh(w http.ResponseWriter, req *http.Request) {

	go UpdateWinds()
}

func Navigate(w http.ResponseWriter, req *http.Request) {
	//runtime.SetCPUProfileRate(300)
	//defer profile.Start().Stop()
	//defer profile.Start(profile.MemProfile).Stop()

	//params := mux.Vars(req)
	var gonav GoNav
	_ = json.NewDecoder(req.Body).Decode(&gonav)

	logger.infof("Navigate '%s' every '%.2f' stop %t\n", gonav.Race.Name, gonav.Delta, gonav.Stop)

	winchMalus := 5.0
	if gonav.Winch {
		winchMalus = 1.25
	}

	start := time.Now()

	deltas := map[int]float64{
		6:    1.0 / 6.0,
		12:   0.5,
		48:   1.0,
		72:   3.0,
		9999: 6.0}

	isos := Run(gonav.Expes, &l, winds, &x, gonav.Start, gonav.Bearing, gonav.CurrentSail, gonav.Race, gonav.Delta, deltas, gonav.MaxDuration, gonav.Delay, gonav.Sail, gonav.Foil, gonav.Hull, winchMalus, gonav.Stop, positionPool)

	delta := time.Now().Sub(start)
	logger.infoln("Navigation took", delta)

	json.NewEncoder(w).Encode(isos)
}

func BoatLines(w http.ResponseWriter, req *http.Request) {
	var gonav GoNav
	_ = json.NewDecoder(req.Body).Decode(&gonav)

	logger.infof("Boatlines '%s' every '%.2f'\n", gonav.Race.Name, gonav.Delta)

	winchMalus := 5.0
	if gonav.Winch {
		winchMalus = 1.25
	}

	start := time.Now()

	lines := GetBoatLines(gonav.Expes, winds, gonav.Start, gonav.Bearing, gonav.CurrentSail, gonav.Race, 1.0, gonav.Delay, gonav.Sail, gonav.Foil, gonav.Hull, winchMalus, positionPool)

	delta := time.Now().Sub(start)
	logger.infoln("Boatlines took", delta)

	json.NewEncoder(w).Encode(lines)
}

func TestLand(w http.ResponseWriter, req *http.Request) {
	isos := RunTestIsLand(&l)

	json.NewEncoder(w).Encode(isos)
}

func Healthz(w http.ResponseWriter, req *http.Request) {
	type health struct {
		Status string `json:"status"`
	}

	json.NewEncoder(w).Encode(health{Status: "Ok"})
}

var l Land
var winds map[string][]*wind.Wind
var x xmpp.Xmpp
var lock = sync.RWMutex{}
var positionPool *sync.Pool

var logger *Logger

func LoadWinds() {
	logger.infoln("Load winds")
	lock.Lock()
	winds = wind.LoadAll2()
	lock.Unlock()

	/*    for d := 0 ; d < 6 ; d++ {
	          Run(&l, winds, &x, LatLon{Lat: 40.430225, Lon: -73.9064}, 57, 1, load(), 3, 480, d, 7, true, true, 5.0, 1, false)
	      }
	      for d := 6 ; d < 36 ; d+=6 {
	          Run(&l, winds, &x, LatLon{Lat: 40.430225, Lon: -73.9064}, 57, 1, load(), 3, 480, d, 7, true, true, 5.0, 1, false)
	      }
	      for d := 36 ; d < 192 ; d+=24 {
	          Run(&l, winds, &x, LatLon{Lat: 40.430225, Lon: -73.9064}, 57, 1, load(), 3, 480, d, 7, true, true, 5.0, 1, false)
	      }
	*/
}

func UpdateWinds() {
	lock.Lock()
	wind.Merge(winds)
	lock.Unlock()
}

func main() {

	logger = &Logger{debug: false}

	fs := flag.NewFlagSet("nav-server", flag.ExitOnError)
	var (
		xmppHost     = fs.String("xmpp-host", "", "")
		xmppJid      = fs.String("xmpp-jid", "", "")
		xmppPassword = fs.String("xmpp-password", "", "")
		xmppTo       = fs.String("xmpp-to", "", "")
	)
	ff.Parse(fs, os.Args[1:], ff.WithEnvVarNoPrefix())

	flag.Parse()

	positionPool = &sync.Pool{
		New: func() interface{} {
			return new(Position)
		},
	}

	x = xmpp.Xmpp{Config: xmpp.Config{Host: *xmppHost, Jid: *xmppJid, Password: *xmppPassword, To: *xmppTo}}

	logger.infoln("Load lands")
	l = InitLand()

	LoadWinds()

	s := gocron.NewScheduler()
	jobxx := s.Every(15).Seconds()
	jobxx.Do(UpdateWinds)
	//    job04 := s.Every(1).Day().At("05:00")
	//    job04.Do(LoadWinds)
	//    job10 := s.Every(1).Day().At("11:00")
	//    job10.Do(LoadWinds)
	//    job16 := s.Every(1).Day().At("17:00")
	//    job16.Do(LoadWinds)
	//    job22 := s.Every(1).Day().At("23:00")
	//    job22.Do(LoadWinds)
	go s.Start()

	logger.infoln("Start server")

	router := mux.NewRouter().StrictSlash(true)
	router.HandleFunc("/nav/run", Navigate).Methods("POST")
	router.HandleFunc("/nav/refresh", Refresh).Methods("GET")
	router.HandleFunc("/nav/expes", Expes).Methods("GET")
	router.HandleFunc("/nav/test", TestLand).Methods("POST")
	router.HandleFunc("/nav/boatlines", BoatLines).Methods("POST")
	router.HandleFunc("/nav/-/healthz", Healthz).Methods("GET")
	log.Fatal(http.ListenAndServe(":8888", router))

}
