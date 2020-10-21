package main

import (
	"testing"
)

func TestIsInIceLimits(t *testing.T) {

	// {
	// 	"lat": -56,
	// 	"lon": -180
	// },
	// {
	// 	"lat": -56.4167,
	// 	"lon": -175
	// },
	// {
	// 	"lat": -56.25,
	// 	"lon": -170
	// },
	// {
	// 	"lat": -55.6667,
	// 	"lon": -165
	// },
	// {
	// 	"lat": -54.75,
	// 	"lon": -160
	// },
	// {
	// 	"lat": -53.9167,
	// 	"lon": -155
	// },
	// {
	// 	"lat": -53.6667,
	// 	"lon": -150
	// },
	// {
	// 	"lat": -53.6667,
	// 	"lon": -145
	// },
	// {
	// 	"lat": -53.6667,
	// 	"lon": -140
	// },
	// {
	// 	"lat": -53.6667,
	// 	"lon": -135
	// },
	// {
	// 	"lat": -53.9167,
	// 	"lon": -130
	// },
	// {
	// 	"lat": -54.0833,
	// 	"lon": -125
	// },
	// {
	// 	"lat": -54.25,
	// 	"lon": -120
	// },
	// {
	// 	"lat": -54.6667,
	// 	"lon": -115
	// },
	// {
	// 	"lat": -55.25,
	// 	"lon": -110
	// },
	// {
	// 	"lat": -55.75,
	// 	"lon": -105
	// },
	// {
	// 	"lat": -56.25,
	// 	"lon": -100
	// },
	// {
	// 	"lat": -57,
	// 	"lon": -95
	// },
	// {
	// 	"lat": -57.75,
	// 	"lon": -90
	// },
	// {
	// 	"lat": -58.5,
	// 	"lon": -85
	// },
	// {
	// 	"lat": -59,
	// 	"lon": -80
	// },
	// {
	// 	"lat": -58.75,
	// 	"lon": -75
	// },
	// {
	// 	"lat": -58,
	// 	"lon": -70
	// },
	// {
	// 	"lat": -57,
	// 	"lon": -65
	// },
	// {
	// 	"lat": -56,
	// 	"lon": -60
	// },
	// {
	// 	"lat": -55,
	// 	"lon": -55
	// },
	// {
	// 	"lat": -52.25,
	// 	"lon": -50
	// },
	// {
	// 	"lat": -49,
	// 	"lon": -45
	// },
	// {
	// 	"lat": -48,
	// 	"lon": -40
	// },
	// {
	// 	"lat": -46,
	// 	"lon": -35
	// },
	// {
	// 	"lat": -45,
	// 	"lon": -30
	// },
	// {
	// 	"lat": -44,
	// 	"lon": -25
	// },
	// {
	// 	"lat": -43.5,
	// 	"lon": -20
	// },
	// {
	// 	"lat": -43,
	// 	"lon": -15
	// },

	iceLimits := IceLimits{
		South: []LatLon{
			LatLon{Lat: -43, Lon: -10},
			LatLon{Lat: -43.5, Lon: -5},
			LatLon{Lat: -43.5, Lon: 0},
			LatLon{Lat: -43.6667, Lon: 5},
			LatLon{Lat: -43.8333, Lon: 10},
			LatLon{Lat: -44.0833, Lon: 15},
			LatLon{Lat: -44.4167, Lon: 20},
			LatLon{Lat: -44.8333, Lon: 25},
			LatLon{Lat: -45.3333, Lon: 30},
			LatLon{Lat: -45.9167, Lon: 35},
			LatLon{Lat: -46.5833, Lon: 40},
			LatLon{Lat: -47.25, Lon: 45},
			LatLon{Lat: -47.9167, Lon: 50},
			LatLon{Lat: -48.75, Lon: 55},
			LatLon{Lat: -49.6667, Lon: 60},
			LatLon{Lat: -50.5833, Lon: 65},
			LatLon{Lat: -50.9167, Lon: 70},
			LatLon{Lat: -50.9167, Lon: 75},
			LatLon{Lat: -50.6667, Lon: 80},
			LatLon{Lat: -50.4167, Lon: 85},
			LatLon{Lat: -50, Lon: 90},
			LatLon{Lat: -49, Lon: 95},
			LatLon{Lat: -47.75, Lon: 100},
			LatLon{Lat: -46, Lon: 105},
			LatLon{Lat: -46, Lon: 110},
			LatLon{Lat: -46, Lon: 115},
			LatLon{Lat: -46, Lon: 120},
			LatLon{Lat: -48.8333, Lon: 125},
			LatLon{Lat: -50.75, Lon: 130},
			LatLon{Lat: -51.9167, Lon: 135},
			LatLon{Lat: -52.25, Lon: 140},
			LatLon{Lat: -52.4167, Lon: 145},
			LatLon{Lat: -52.75, Lon: 150},
			LatLon{Lat: -53.25, Lon: 155},
			LatLon{Lat: -53.8333, Lon: 160},
			LatLon{Lat: -54.25, Lon: 165},
			LatLon{Lat: -54.8333, Lon: 170},
			LatLon{Lat: -56.0, Lon: 180}},
		MaxLat: 90.0,
		MinLat: 90.0}

	ice := iceLimits.isInIceLimits(&LatLon{Lat: -50.0, Lon: 30.0})
	if !ice {
		t.Errorf("isInIceLimits(-50, 30) = (%t); want (true)", ice)
	}

	ice = iceLimits.isInIceLimits(&LatLon{Lat: -60.0, Lon: 70.0})
	if !ice {
		t.Errorf("isInIceLimits(-70, 60) = (%t); want (true)", ice)
	}
}
