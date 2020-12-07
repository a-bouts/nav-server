package api

import (
	"encoding/json"
	"fmt"
	"net"
	"net/http"
	"strings"
	"sync"
	"time"

	log "github.com/sirupsen/logrus"

	"github.com/a-bouts/nav-server/api/model"
	"github.com/a-bouts/nav-server/land"
	"github.com/a-bouts/nav-server/latlon"
	"github.com/a-bouts/nav-server/race"
	"github.com/a-bouts/nav-server/route"
	"github.com/a-bouts/nav-server/wind"
	"github.com/a-bouts/nav-server/xmpp"
	"github.com/gorilla/mux"
	"github.com/pkg/profile"
)

type server struct {
	cpuprofile   bool
	l            *land.Land
	w            *wind.Winds
	x            *xmpp.Xmpp
	positionPool *sync.Pool
}

func InitServer(cpuprofile bool, l *land.Land, w *wind.Winds, x *xmpp.Xmpp) *mux.Router {

	router := mux.NewRouter().StrictSlash(true)

	s := server{cpuprofile: cpuprofile,
		l: l,
		w: w,
		x: x,
		positionPool: &sync.Pool{
			New: func() interface{} {
				return new(route.Position)
			},
		},
	}

	api := router.PathPrefix("/").Subrouter()

	api.HandleFunc("/nav/-/healthz", s.healthz).Methods(http.MethodGet)
	api.HandleFunc("/nav/run", s.routeOld).Methods("POST")
	api.HandleFunc("/nav/expes", s.getExpes).Methods("GET")
	api.HandleFunc("/nav/boatlines", s.sneakOld).Methods("POST")

	apiV1 := router.PathPrefix("/route/api/v1").Subrouter()
	apiV1.HandleFunc("/route", s.route).Methods("POST")
	apiV1.HandleFunc("/expes", s.getExpes).Methods("GET")
	apiV1.HandleFunc("/sneak", s.sneak).Methods("POST")

	return router
}

func (s *server) healthz(w http.ResponseWriter, r *http.Request) {
	type health struct {
		Status string `json:"status"`
	}

	json.NewEncoder(w).Encode(health{Status: "Ok"})
}

func (s *server) getExpes(w http.ResponseWriter, req *http.Request) {

	expes := []string{
		"new-polars",
		"progressive-intervales",
		"sqrt-dist-from",
		"optim",
		"max-dist"}

	json.NewEncoder(w).Encode(expes)
}

type GoNav struct {
	Expes       map[string]bool `json:"expes"`
	Start       latlon.LatLon   `json:"start"`
	Bearing     int             `json:"bearing"`
	CurrentSail byte            `json:"currentSail"`
	Race        race.Race       `json:"race"`
	Delta       float64         `json:"delta"`
	MaxDuration float64         `json:"maxDuration"`
	Delay       int             `json:"delay"`
	StartTime   time.Time       `json:"startTime"`
	Sail        int             `json:"sail"`
	Foil        bool            `json:"foil"`
	Hull        bool            `json:"hull"`
	Winch       bool            `json:"winch"`
	Malus       float64         `json:"malus"`
	Stop        bool            `json:"stop"`
}

func (s *server) route(w http.ResponseWriter, req *http.Request) {
	if s.cpuprofile {
		//runtime.SetCPUProfileRate(300)
		defer profile.Start().Stop()
	}
	//defer profile.Start(profile.MemProfile).Stop()

	fields := log.Fields{
		"action": "route",
	}
	if ip, err := getIp(req); err == nil {
		fields["IP"] = ip
	}
	requestLogger := log.WithFields(fields)

	var r model.Route
	_ = json.NewDecoder(req.Body).Decode(&r)

	requestLogger.Infof("Route '%s' from '%s' every '%.2f' stop %t\n", r.Race.Name, r.StartTime.String(), r.Params.Delta, r.Params.Stop)

	start := time.Now()

	deltas := map[int]float64{
		6:    1.0 / 6.0,
		12:   0.5,
		48:   1.0,
		72:   3.0,
		9999: 6.0}

	isos := route.Run(r, s.l, s.w, s.x, deltas, s.positionPool)

	delta := time.Now().Sub(start)
	requestLogger.Infof("Route took %s", delta.String())

	json.NewEncoder(w).Encode(isos)
}

func (s *server) sneak(w http.ResponseWriter, req *http.Request) {

	var r model.Route
	_ = json.NewDecoder(req.Body).Decode(&r)

	log.Infof("Sneak '%s' every '%.2f'\n", r.Race.Name, r.Params.Delta)

	start := time.Now()

	lines := route.GetBoatLines(r, s.w, s.positionPool)

	delta := time.Now().Sub(start)
	log.Infof("Sneak took %s", delta.String())

	json.NewEncoder(w).Encode(lines)
}

func (s *server) routeOld(w http.ResponseWriter, req *http.Request) {
	if s.cpuprofile {
		//runtime.SetCPUProfileRate(300)
		defer profile.Start().Stop()
	}
	//defer profile.Start(profile.MemProfile).Stop()

	fields := log.Fields{
		"action": "route",
	}
	if ip, err := getIp(req); err == nil {
		fields["IP"] = ip
	}
	requestLogger := log.WithFields(fields)

	var gonav GoNav
	_ = json.NewDecoder(req.Body).Decode(&gonav)

	r := model.Route{
		Params: model.Params{
			Expes:       gonav.Expes,
			Stop:        gonav.Stop,
			Delta:       gonav.Delta,
			MaxDuration: gonav.MaxDuration,
		},
		StartTime:   gonav.StartTime,
		Start:       gonav.Start,
		Bearing:     gonav.Bearing,
		CurrentSail: gonav.CurrentSail,
		Race:        gonav.Race,
		Options: model.Options{
			Sail:  gonav.Sail,
			Foil:  gonav.Foil,
			Hull:  gonav.Hull,
			Winch: gonav.Winch,
		},
	}

	requestLogger.Infof("Route '%s' from '%s' every '%.2f' stop %t\n", gonav.Race.Name, gonav.StartTime.String(), gonav.Delta, gonav.Stop)

	start := time.Now()

	deltas := map[int]float64{
		6:    1.0 / 6.0,
		12:   0.5,
		48:   1.0,
		72:   3.0,
		9999: 6.0}

	isos := route.Run(r, s.l, s.w, s.x, deltas, s.positionPool)

	delta := time.Now().Sub(start)
	requestLogger.Infof("Route took %s", delta.String())

	json.NewEncoder(w).Encode(isos)
}

func (s *server) sneakOld(w http.ResponseWriter, req *http.Request) {

	var gonav GoNav
	_ = json.NewDecoder(req.Body).Decode(&gonav)

	r := model.Route{
		Params: model.Params{
			Expes:       gonav.Expes,
			Stop:        gonav.Stop,
			Delta:       gonav.Delta,
			MaxDuration: gonav.MaxDuration,
		},
		StartTime:   gonav.StartTime,
		Start:       gonav.Start,
		Bearing:     gonav.Bearing,
		CurrentSail: gonav.CurrentSail,
		Race:        gonav.Race,
		Options: model.Options{
			Sail:  gonav.Sail,
			Foil:  gonav.Foil,
			Hull:  gonav.Hull,
			Winch: gonav.Winch,
		},
	}

	log.Infof("Sneak '%s' every '%.2f'\n", gonav.Race.Name, gonav.Delta)

	start := time.Now()

	lines := route.GetBoatLines(r, s.w, s.positionPool)

	delta := time.Now().Sub(start)
	log.Infof("Sneak took %s", delta.String())

	json.NewEncoder(w).Encode(lines)
}

func getIp(r *http.Request) (string, error) {
	//Get IP from the X-REAL-IP header
	ip := r.Header.Get("X-REAL-IP")
	netIP := net.ParseIP(ip)
	if netIP != nil {
		return ip, nil
	}

	//Get IP from X-FORWARDED-FOR header
	ips := r.Header.Get("X-FORWARDED-FOR")
	splitIps := strings.Split(ips, ",")
	for _, ip := range splitIps {
		netIP := net.ParseIP(ip)
		if netIP != nil {
			return ip, nil
		}
	}

	//Get IP from RemoteAddr
	ip, _, err := net.SplitHostPort(r.RemoteAddr)
	if err != nil {
		return "", err
	}
	netIP = net.ParseIP(ip)
	if netIP != nil {
		return ip, nil
	}
	return "", fmt.Errorf("No valid ip found")
}
