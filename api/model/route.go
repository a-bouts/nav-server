package model

import (
	"time"

	"github.com/a-bouts/nav-server/latlon"
	"github.com/a-bouts/nav-server/race"
)

type Route struct {
	Params      Params        `json:"params"`
	StartTime   time.Time     `json:"startTime"`
	Start       latlon.LatLon `json:"start"`
	Bearing     int           `json:"bearing"`
	CurrentSail byte          `json:"currentSail"`
	Race        race.Race     `json:"race"`
	Options     Options       `json:"options"`
}

type Options struct {
	Sail     byte `json:"sail"`
	Foil     bool `json:"foil"`
	Hull     bool `json:"hull"`
	Winch    bool `json:"winch"`
	AutoSail bool `json:"autoSail"`
}

type Params struct {
	Expes       map[string]bool `json:"expes"`
	Stop        bool            `json:"stop"`
	Delta       float64         `json:"delta"`
	Accuracy    int             `json:"accuracy"`
	MaxDuration float64         `json:"maxDuration"`
	Delay       int             `json:"delay"`
	Deltas      map[int]float64 `json:"deltas"`
}
