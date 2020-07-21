package main

import (
	"math"
	"testing"
)

func TestWrap360(t *testing.T) {
	a := wrap360(-1.0)
	if a != 359.0 {
		t.Errorf("wrap360(-1) = %f; want 359.0", a)
	}
	b := wrap360(361.0)
	if b != 1.0 {
		t.Errorf("wrap360(-361.0) = %f; want 1.0", b)
	}
}

func TestRhumbDistanceTo(t *testing.T) {
	p1 := LatLon{Lat: 51.127, Lon: 1.338}
	p2 := LatLon{Lat: 50.964, Lon: 1.853}
	d := LatLonSpherical{}.DistanceTo(p1, p2)
	if math.Round(d) != 40308 {
		t.Errorf("{%f,%f}.rhumbDistanceTo({%f,%f}) = %f; want 40308", p1.Lat, p1.Lon, p2.Lat, p2.Lon, d)
	}
}

func TestRhumbBearingTo(t *testing.T) {
	p1 := LatLon{Lat: 51.127, Lon: 1.338}
	p2 := LatLon{Lat: 50.964, Lon: 1.853}
	b := LatLonSpherical{}.BearingTo(p1, p2)
	if math.Round(b*10)/10 != 116.7 {
		t.Errorf("{%f,%f}.rhumbBearingTo({%f,%f}) = %f; want 116.7", p1.Lat, p1.Lon, p2.Lat, p2.Lon, b)
	}
}

func TestRhumbDestinationPoint(t *testing.T) {
	p1 := LatLon{Lat: 51.127, Lon: 1.338}
	p2 := LatLonSpherical{}.Destination(p1, 40300.0, 116.7)
	if math.Round(p2.Lat*10000)/10000 != 50.9642 && math.Round(p2.Lon*10000)/10000 != 1.8530 {
		t.Errorf("{%f,%f}.rhumbDestinationPoint(40300.0, 116.7) = {%f,%f}; want {50.9642,1.8530}", p1.Lat, p1.Lon, p2.Lat, p2.Lon)
	}
}
