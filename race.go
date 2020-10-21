package main

import (
	"encoding/json"
	"fmt"
	"io/ioutil"
)

type RaceWaypoint struct {
	Name      string        `json:"name"`
	Latlons   []LatLon      `json:"latlons"`
	Validated bool          `json:"validated"`
	ToAvoid   [][][]float64 `json:"toAvoid"`
}

type IceLimits struct {
	North  []LatLon `json:"north"`
	South  []LatLon `json:"south"`
	MaxLat int      `json:"maxLat"`
	MinLat int      `json:"minLat"`
}

type Race struct {
	Name      string         `json:"name"`
	Polars    string         `json:"polars"`
	Boat      string         `json:"boat"`
	Start     LatLon         `json:"start"`
	Waypoints []RaceWaypoint `json:"waypoints"`
	IceLimits IceLimits      `json:"ice_limits"`
}

type Races struct {
	races map[string]Race
}

func test() {
	r := Race{Name: "test", Start: LatLon{Lat: 12.2, Lon: 14.3}}
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

func (iceLimits *IceLimits) isInIceLimits(latLon *LatLon) bool {

	if float64(iceLimits.MinLat) < latLon.Lat && latLon.Lat < float64(iceLimits.MaxLat) {
		return false
	}

	if latLon.Lat > 0.0 {
		for i := 0; i < len(iceLimits.North)-1; i++ {
			if latLon.Lon >= iceLimits.North[i].Lon && latLon.Lon <= iceLimits.North[i+1].Lon {
				lat := (iceLimits.North[i+1].Lon-latLon.Lon)/(iceLimits.North[i+1].Lon-iceLimits.North[i].Lon)*(iceLimits.North[i+1].Lat-iceLimits.North[i].Lat) + iceLimits.North[i].Lat
				if latLon.Lat >= lat {
					return true
				}
				return false
			}
		}
	} else {
		for i := 0; i < len(iceLimits.South)-1; i++ {
			if latLon.Lon >= iceLimits.South[i].Lon && latLon.Lon <= iceLimits.South[i+1].Lon {
				lat := (iceLimits.South[i+1].Lon-latLon.Lon)/(iceLimits.South[i+1].Lon-iceLimits.South[i].Lon)*(iceLimits.South[i+1].Lat-iceLimits.South[i].Lat) + iceLimits.South[i].Lat
				if latLon.Lat <= lat {
					return true
				}
				return false
			}
		}
	}

	return false
}
