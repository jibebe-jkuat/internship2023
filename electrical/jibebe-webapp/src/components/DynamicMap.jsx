import {
  MapContainer,
  TileLayer,
  Marker,
  Popup,
  Polygon,
  Polyline,
} from "react-leaflet";
import * as turf from "@turf/turf";

import Leaflet from "leaflet";

import "leaflet/dist/leaflet.css";
import { useEffect, useState } from "react";
import { Button } from "@mantine/core";
import { notifications } from "@mantine/notifications";

export default function DynamicMap({ mapCenter, getMenuState }) {
  const [polygonCoords, setPolygonCoords] = useState([]);
  const [map, setMap] = useState(null);
  const [menuState, setMenuState] = useState(false);
  const [tracks, setTracks] = useState([]);
  const [loading, setLoading] = useState(false);

  useEffect(() => {
    if (map) {
      console.log("Map present");
      map.on("click", function (e) {
        console.log([e.latlng.lat, e.latlng.lng]);

        setMenuState(true);

        setPolygonCoords((prevCoords) => [
          ...prevCoords,
          [e.latlng.lat, e.latlng.lng],
        ]);
      });
    }
  }, [map]);

  const handleRemoveCoordinate = () => {
    if (polygonCoords.length == 0) {
      setMenuState(false);
      return;
    }
    setPolygonCoords((prevCoords) => {
      const newCoords = [...prevCoords];
      newCoords.pop();

      return newCoords;
    });
  };

  const handleGenerateTracks = (polygonCoordinates) => {
    const trackWidth = 0.000075; // 1.5 meters in degrees

    // Create a turf polygon feature from the input polygon coordinates
    const turfPolygon = turf.polygon([
      [...polygonCoordinates, polygonCoordinates[0]],
    ]);

    // Calculate the bounding box of the polygon
    const bbox = turf.bbox(turfPolygon);

    const [minX, minY, maxX, maxY] = bbox;

    for (let i = minX; i <= maxX; i += trackWidth) {
      for (let j = minY; j <= maxY; j += trackWidth) {
        // Create a turf lineString feature for the track
        const turfLineString = turf.lineString([
          [i, j],
          [i + trackWidth, j],
        ]);

        // Check if the track intersects with the polygon
        if (
          turf.lineOverlap(turfLineString, turfPolygon)?.features.length == 0
        ) {
          // Convert the turf lineString to an array of coordinates
          const trackCoordinates = turfLineString.geometry.coordinates;

          const [_p1, _p2] = trackCoordinates;
          const [_lat1, _lng1] = _p1;
          const [_lat2, _lng2] = _p2;

          const differential = 0.0001;

          console.log(
            turf.difference(
              turf.polygon([
                [
                  [_lat1, _lng1],
                  [_lat2, _lng2],
                  [_lat1, _lng2 + differential],
                  [_lat2, _lng2 + differential],
                  [_lat1, _lng1],
                ],
              ]),
              turf.polygon([[...polygonCoordinates, polygonCoordinates[0]]])
            )
          );

          // Add the track coordinates to the tracks array
          setTracks((prevTracks) => [...prevTracks, trackCoordinates]);
        }
      }
    }
  };

  let marker = new Leaflet.Icon({
    iconUrl: "/tractor.png",
    iconSize: [45, 35],
    iconAnchor: [32, 64],
    popupAnchor: null,
    shadowUrl: null,
    shadowSize: null,
    shadowAnchor: null,
  });

  return (
    <span>
      <MapContainer
        center={mapCenter}
        zoom={17}
        scrollWheelZoom={false}
        style={{ height: "100%", width: "100%", zIndex: 79 }}
        whenCreated={(map) => setMap(map)}
      >
        <TileLayer
          attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
          url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        />
        <Marker position={mapCenter} icon={marker}>
          <Popup>
            A pretty CSS3 popup. <br /> Easily customizable.
          </Popup>
        </Marker>
        {polygonCoords.length > 0 && (
          <Polygon
            pathOptions={{ color: "purple" }}
            positions={polygonCoords}
          />
        )}
        {/* Render the tracks */}
        {tracks.map((track, index) => (
          <Polyline key={index} positions={track} />
        ))}
      </MapContainer>
      {menuState && (
        <div
          className=" flex gap-3 absolute bottom-0 w-full p-4 bg-white"
          style={{ zIndex: 99 }}
        >
          <Button
            color="red"
            variant="outline"
            onClick={handleRemoveCoordinate}
          >
            {" "}
            &larr; Back
          </Button>
          <Button
            onClick={() => handleGenerateTracks(polygonCoords)}
            color="green"
            loading={loading}
          >
            Generate tracks
          </Button>
          <Button color="red" variant="outline" onClick={() => setTracks([])}>
            {" "}
            Clear tracks
          </Button>
        </div>
      )}
    </span>
  );
}
