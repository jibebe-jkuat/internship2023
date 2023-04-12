import { Button } from "@mantine/core";
import dynamic from "next/dynamic";
import { useState } from "react";

const DynamicMap = dynamic(() => import("./DynamicMap"), {
  ssr: false,
});

const Map = ({ mapCenter }) => {
  return (
    <div style={{ height: "100%" }} className="relative">
      <DynamicMap mapCenter={mapCenter} />
    </div>
  );
};

export default Map;
