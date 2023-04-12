import { Badge, Button, Divider, Input, Select, Text } from "@mantine/core";

import {
  IconBatteryCharging,
  IconChartInfographic,
  IconClock,
  IconFocus2,
  IconLocation,
  IconTemperature,
  IconX,
} from "@tabler/icons";
import { useEffect, useMemo, useRef, useState } from "react";
import useDimensions from "@/hooks/useDimensions.js";

import { buildStyles, CircularProgressbar } from "react-circular-progressbar";
import "react-circular-progressbar/dist/styles.css";
import { notifications } from "@mantine/notifications";
import Map from "@/components/Map";

const MapView = () => {
  const [center, setCenter] = useState([]);
  const [info, setInfo] = useState(false);
  const [tractorState, setTractorState] = useState("ON");

  useEffect(() => {
    navigator.geolocation.getCurrentPosition(function (position) {
      setCenter(() => [position.coords.latitude, position.coords.longitude]);
    });
  }, []);

  if (center.length < 1) return <p>Loading...</p>;

  return (
    <div className="w-full h-screen relative">
      {/* Header */}
      <div
        style={{ zIndex: 99 }}
        className="flex bg-white p-3 w-full justify-between items-center space-x-6 absolute top-0"
      >
        <img src="/jibebe_logo.PNG" className="h-[48px]" />
        <Select data={["My farm", "Jeremys farm"]} />
      </div>

      {/* Stats */}
      <div className="absolute top-[84px] left-[12px]" style={{ zIndex: 99 }}>
        <Button
          color="green"
          w={48}
          h={48}
          p={0}
          onClick={() => setInfo((info) => !info)}
        >
          <IconChartInfographic />
        </Button>
        <div
          className={
            info
              ? "bg-white p-3 pt-8 mt-[-12px] rounded-md"
              : "bg-white p-3 pt-8 mt-[-12px] rounded-md hidden"
          }
        >
          <Divider label="About tractor" />
          <br />
          <div className="w-[60vw] space-y-3">
            <p className="items-baseline flex space-x-3 font-bold">
              Tractor is
              {tractorState == "OFF" ? (
                <Badge color="red">OFF</Badge>
              ) : (
                <Badge color="green">ON</Badge>
              )}
            </p>
            <div className="flex justify-between">
              <div className="flex space-x-3">
                <IconBatteryCharging color="gray" size={24} />
                <Text>Battery level</Text>
              </div>
              <Text c="green" fw={700} fz={"lg"}>
                70%
              </Text>
            </div>
            <div className="flex justify-between">
              <div className="flex space-x-3">
                <IconClock color="gray" size={20} />
                <Text>Est. time left</Text>
              </div>
              <Text c="orange" fw={700} fz={"lg"}>
                2h 39m
              </Text>
            </div>
            <div className="flex justify-between">
              <div className="flex space-x-3">
                <IconTemperature color="gray" size={20} />
                <Text>Temperature</Text>
              </div>
              <Text c="red" fw={700} fz={"lg"}>
                50 °C
              </Text>
            </div>
          </div>
        </div>
      </div>

      <Map mapCenter={center} />
    </div>
  );
};

export default MapView;
