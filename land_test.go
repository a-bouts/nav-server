package main

import "testing"

func Test(t *testing.T) {
	l := InitLand()
	a := l.IsLand(0.0, 0.0)
	if a {
		t.Errorf("isLand(0.0, 0.0) = ttue; want false")
	}
}
