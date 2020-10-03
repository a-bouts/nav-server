package main

import (
	"encoding/json"
	"fmt"
	"io/ioutil"
)

type Waypoint struct {
	Name      string        `json:"name"`
	Latlons   []LatLon      `json:"latlons"`
	Validated bool          `json:"validated"`
	ToAvoid   [][][]float64 `json:"toAvoid"`
}

type Race struct {
	Name      string     `json:"name"`
	Polars    string     `json:"polars"`
	Boat      string     `json:"boat"`
	Start     LatLon     `json:"start"`
	Waypoints []Waypoint `json:"waypoints"`
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
	fmt.Println(rs)
	return rs[0]
}

func (r Race) IsValidated(index int) bool {
	return r.NextWaypoint(index).Validated
}

func (r Race) HasNextWaypoint(index int) bool {
	return index < len(r.Waypoints)
}

func (r Race) NextWaypoint(index int) Waypoint {
	return r.Waypoints[index]
}

func (r Race) Reached(index int) int {
	return index + 1
}
