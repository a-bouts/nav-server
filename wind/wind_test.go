package wind

import (
	"testing"
	"time"
)

func TestWind(t *testing.T) {

	date, _ := time.Parse("2006010215", "2022010712")

	Init(date, "2022010712.f009")
}
