package race

import (
	"encoding/json"
	"fmt"
	"io/ioutil"

	"github.com/a-bouts/nav-server/latlon"
)

type RaceWaypoint struct {
	Name      string          `json:"name"`
	Latlons   []latlon.LatLon `json:"latlons"`
	Validated bool            `json:"validated"`
	ToAvoid   [][][]float64   `json:"toAvoid"`
	Radius    int             `json:"radius"`
}

type IceLimits struct {
	North  []latlon.LatLon `json:"north"`
	South  []latlon.LatLon `json:"south"`
	MaxLat float64         `json:"maxLat"`
	MinLat float64         `json:"minLat"`
}

type Race struct {
	Name      string         `json:"name"`
	Polars    string         `json:"polars"`
	Boat      string         `json:"boat"`
	Start     latlon.LatLon  `json:"start"`
	Waypoints []RaceWaypoint `json:"waypoints"`
	IceLimits IceLimits      `json:"ice_limits"`
}

type Races struct {
	races map[string]Race
}

func test() {
	r := Race{Name: "test", Start: latlon.LatLon{Lat: 12.2, Lon: 14.3}}
	emp, _ := json.Marshal(r)
	var rs Race
	json.Unmarshal(emp, &rs)
	fmt.Println(string(emp))
}

func load() Race {
	var rs []Race
	content, _ := ioutil.ReadFile("races.json")
	json.Unmarshal(content, &rs)

	return rs[0]
}

func (r Race) IsValidated(index int) bool {
	return r.Waypoints[index].Validated
}

func (r Race) HasNextWaypoint(index int) bool {
	return index < len(r.Waypoints)
}

func (r Race) NextWaypoint(index int) RaceWaypoint {
	return r.Waypoints[index]
}

func (r Race) Reached(index int) int {
	return index + 1
}

func (iceLimits *IceLimits) IsInIceLimits(latLon *latlon.LatLon) bool {

	lon := latLon.Lon
	if lon > 180 {
		lon -= 360
	}
	if lon < -180 {
		lon += 360
	}

	if float64(iceLimits.MinLat) < latLon.Lat && latLon.Lat < float64(iceLimits.MaxLat) {
		return false
	}

	if latLon.Lat > 0.0 {
		if len(iceLimits.North) == 0 {
			return false
		}

		i := int((lon + 180) / 5)
		lat := (lon-iceLimits.North[i].Lon)/(iceLimits.North[i+1].Lon-iceLimits.North[i].Lon)*(iceLimits.North[i+1].Lat-iceLimits.North[i].Lat) + iceLimits.North[i].Lat
		if latLon.Lat >= lat {
			return true
		}
		return false
	} else {
		if len(iceLimits.South) == 0 {
			return false
		}

		i := int((lon + 180) / 5)
		lat := (lon-iceLimits.South[i].Lon)/(iceLimits.South[i+1].Lon-iceLimits.South[i].Lon)*(iceLimits.South[i+1].Lat-iceLimits.South[i].Lat) + iceLimits.South[i].Lat
		if latLon.Lat <= lat {
			return true
		}
		return false
	}

}
