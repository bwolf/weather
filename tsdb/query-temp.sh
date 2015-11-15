#!/bin/sh

curl -G 'http://localhost:8086/query?pretty=true' \
     --data-urlencode "db=weather" \
     --data-urlencode "q=SELECT value FROM temp WHERE station='1';"
