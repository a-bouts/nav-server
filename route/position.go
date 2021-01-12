package route

import (
	"sync"

	"github.com/a-bouts/nav-server/latlon"
)

type Position struct {
	Latlon           latlon.LatLon
	fromDist         float64
	bearing          float64
	twa              float64
	wind             float64
	windSpeed        float64
	boatSpeed        float64
	foil             uint8
	distTo           float64
	previousWindLine *Position
	duration         float64
	navDuration      float64
	doorReached      uint8
	isLand           bool
	sail             byte
	change           bool
	reached          bool
	isInIceLimits    bool
	fromAlternative  bool
}

type positionProvider interface {
	get() *Position
	put(*Position)
}

type positionProviderPool struct {
	pool *sync.Pool
}

func (p positionProviderPool) get() *Position {
	pos := p.pool.Get().(*Position)
	pos.clear()
	return pos
}

func (p positionProviderPool) put(pos *Position) {
	p.pool.Put(pos)
}

type positionProviderNew struct {
}

func (p positionProviderNew) get() *Position {
	return new(Position)
}

func (p positionProviderNew) put(pos *Position) {
}

func (pos *Position) clear() {
	pos.fromDist = 0
	pos.bearing = 0
	pos.twa = 0
	pos.wind = 0
	pos.windSpeed = 0
	pos.boatSpeed = 0
	pos.sail = 0
	pos.foil = 0
	pos.distTo = 0
	pos.previousWindLine = nil
	pos.duration = 0
	pos.navDuration = 0
	pos.isLand = false
	pos.change = false
	pos.reached = false
	pos.doorReached = 0
	pos.isInIceLimits = false
	pos.fromAlternative = false
}
