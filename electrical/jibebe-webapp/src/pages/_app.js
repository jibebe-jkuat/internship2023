import '@/styles/globals.css'

import "../styles/globals.css";
import Head from "next/head";
import { createEmotionCache, MantineProvider } from "@mantine/core";
import { Notifications } from "@mantine/notifications";

function MyApp({ Component, pageProps }) {
  const myCache = createEmotionCache({
    key: "mantine",
    prepend: false,
  });

  return (
    <>
      <Head>
        <title>Jibebe | E-tractor</title>
        <meta
          name="viewport"
          content="minimum-scale=1, initial-scale=1, width=device-width"
        />
      </Head>

      <MantineProvider
        withGlobalStyles
        withNormalizeCSS
        emotionCache={myCache}
        theme={{
          /** Put your mantine theme override here */
          colorScheme: "light",
        }}
      >
        <Notifications />
        <Component {...pageProps} />
      </MantineProvider>
    </>
  );
}

export default MyApp;
