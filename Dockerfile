FROM golang

WORKDIR /go/src/nav-server

COPY . .

RUN cd land && gunzip output.gz

RUN go install -v ./...

CMD ["nav-server"]
