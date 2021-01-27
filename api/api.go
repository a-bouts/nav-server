package api

import (
	"encoding/json"
	"fmt"
	"net"
	"net/http"
	"strconv"
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
	p            wind.Providers
	x            *xmpp.Xmpp
	positionPool *sync.Pool
}

func InitServer(cpuprofile bool, l *land.Land, p wind.Providers, x *xmpp.Xmpp) *mux.Router {

	router := mux.NewRouter().StrictSlash(true)

	s := server{cpuprofile: cpuprofile,
		l: l,
		p: p,
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
	apiV1.HandleFunc("/route", s.routeV1).Methods("POST")
	apiV1.HandleFunc("/expes", s.getExpes).Methods("GET")
	apiV1.HandleFunc("/sneak", s.sneakV1).Methods("POST")
	apiV1.HandleFunc("/wind/{stamp}/{file}/{lat}/{lon}", s.wind).Methods("GET")

	apiV2 := router.PathPrefix("/route/api/v2").Subrouter()
	apiV2.HandleFunc("/route", s.route).Methods("POST")
	apiV2.HandleFunc("/expes", s.getExpes).Methods("GET")
	apiV2.HandleFunc("/sneak", s.sneak).Methods("POST")
	apiV2.HandleFunc("/wind/{provider}/{stamp}/{file}/{lat}/{lon}", s.wind).Methods("GET")

	return router
}

func (s *server) wind(w http.ResponseWriter, r *http.Request) {
	provider := mux.Vars(r)["provider"]
	stamp := mux.Vars(r)["stamp"]
	file := mux.Vars(r)["file"]

	lat, err := strconv.ParseFloat(mux.Vars(r)["lat"], 64)
	if err != nil {
		w.WriteHeader(http.StatusBadRequest)
		return
	}
	lon, err := strconv.ParseFloat(mux.Vars(r)["lon"], 64)
	if err != nil {
		w.WriteHeader(http.StatusBadRequest)
		return
	}

	type windResult struct {
		Wind  float64 `json:"wind"`
		Speed float64 `json:"speed"`
	}

	for _, wi := range s.p[provider].Winds(stamp) {
		if wi.File == file {
			var res windResult
			res.Wind, res.Speed = wind.Interpolate([]*wind.Wind{wi}, nil, lat, lon, 0)
			res.Speed *= 1.9438444924406

			log.Infof("Wind %s (%f,%f) : %.1f° %.1f kt", file, lat, lon, res.Wind, res.Speed)

			json.NewEncoder(w).Encode(res)
			return
		}
	}
	w.WriteHeader(http.StatusNotFound)
}

func (s *server) healthz(w http.ResponseWriter, r *http.Request) {
	type health struct {
		Status string `json:"status"`
	}

	json.NewEncoder(w).Encode(health{Status: "Ok"})
}

func (s *server) getExpes(w http.ResponseWriter, req *http.Request) {

	expes := []string{
		"sqrt-dist-from",
		"optim",
		"max-dist",
		"alternatives",
		"vent1",
		"vent2",
		"vent3",
		"vent4",
	}

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
	Sail        byte            `json:"sail"`
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

	if r.Params.Accuracy <= 0 || r.Params.Accuracy > 5 {
		r.Params.Accuracy = 3
	}

	requestLogger.Infof("Route '%s' from '%s' at '%d' every '%.2f' stop %t\n", r.Race.Name, r.StartTime.String(), r.Params.Accuracy, r.Params.Delta, r.Params.Stop)

	start := time.Now()

	deltas := map[int]float64{
		6:    1.0 / 6.0,
		12:   0.5,
		48:   1.0,
		72:   3.0,
		9999: 6.0}

	isos, ops := route.Run(r, s.l, s.p[r.Provider], s.x, deltas, s.positionPool)

	delta := time.Now().Sub(start)
	requestLogger.Infof("Route took %s (%d)", delta.String(), ops)

	json.NewEncoder(w).Encode(isos)
}

func (s *server) routeV1(w http.ResponseWriter, req *http.Request) {
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

	if r.Params.Accuracy <= 0 || r.Params.Accuracy > 5 {
		r.Params.Accuracy = 3
	}

	requestLogger.Infof("Route '%s' from '%s' at '%d' every '%.2f' stop %t\n", r.Race.Name, r.StartTime.String(), r.Params.Accuracy, r.Params.Delta, r.Params.Stop)

	start := time.Now()

	deltas := map[int]float64{
		6:    1.0 / 6.0,
		12:   0.5,
		48:   1.0,
		72:   3.0,
		9999: 6.0}

	if r.Race.Polars == "vg" {
		deltas = map[int]float64{
			24:   1.0 / 6.0,
			48:   0.5,
			72:   1.0,
			9999: 3.0}
	}

	isos, ops := route.Run(r, s.l, s.p["noaa"], s.x, deltas, s.positionPool)

	delta := time.Now().Sub(start)
	requestLogger.Infof("Route took %s (%d)", delta.String(), ops)

	json.NewEncoder(w).Encode(isos)
}

func (s *server) sneak(w http.ResponseWriter, req *http.Request) {

	var r model.Route
	_ = json.NewDecoder(req.Body).Decode(&r)

	log.Infof("Sneak '%s' for %d hours\n", r.Race.Name, int(r.Params.MaxDuration))
	log.Debug(r.Options)

	start := time.Now()

	lines := route.EvalSneak(r, s.p[r.Provider], s.positionPool)

	delta := time.Now().Sub(start)
	log.Infof("Sneak took %s", delta.String())

	json.NewEncoder(w).Encode(lines)
}

func (s *server) sneakV1(w http.ResponseWriter, req *http.Request) {

	var r model.Route
	_ = json.NewDecoder(req.Body).Decode(&r)

	log.Infof("Sneak '%s' for %d hours\n", r.Race.Name, int(r.Params.MaxDuration))
	log.Debug(r.Options)

	start := time.Now()

	lines := route.EvalSneak(r, s.p["noaa"], s.positionPool)

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
		Bearing:     float64(gonav.Bearing),
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

	isos, ops := route.Run(r, s.l, s.p["noaa"], s.x, deltas, s.positionPool)

	delta := time.Now().Sub(start)
	requestLogger.Infof("Route took %s (%d)", delta.String(), ops)

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
		Bearing:     float64(gonav.Bearing),
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

	lines := route.GetBoatLines(r, s.p["noaa"], s.positionPool)

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
