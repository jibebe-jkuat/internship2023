export default function Test() {
  return (
    <>
      <Navigation />
      <Hero />
      <About />
    </>
  );
}

const Navigation = () => {
  return (
    // Navigation
    <header class="fixed top-0 z-40 w-full" aria-label="Banner">
      <div class="flex justify-between md:items-center flex-row relative">
        {/* <!-- background --> */}
        <div
          id="navigation-background"
          class="w-full h-[5.1rem] -translate-y-[2.4rem] md:-translate-y-[0.9rem] bg-neutrals-900/80 absolute transition-all duration-500"
        ></div>
        {/* 
    <!-- primary section --> */}
        <div class="right-1/2 translate-x-1/2 w-10/12 2xl:w-[80%] absolute">
          <div class="flex justify-between md:items-center flex-row relative">
            {/* <!-- logo --> */}
            <a
              class="flex items-center font-display py-3 text-xs md:text-base uppercase"
              href="/"
            >
              <strong class="mr-1">KINUT</strong>Labs
            </a>

            {/* <!-- mobile navigation button --> */}
            <button
              id="toggle-menu"
              aria-label="Toggle the navigation menu"
              class="flex items-center justify-center cursor-pointer md:hidden"
            >
              <svg
                id="toggle-menu-open"
                aria-hidden="true"
                xmlns="http://www.w3.org/2000/svg"
                width="24"
                height="24"
                class="w-6 h-6"
                fill="none"
                viewBox="0 0 24 24"
                stroke="currentColor"
                stroke-width="2"
              >
                <path
                  stroke-linecap="round"
                  stroke-linejoin="round"
                  d="M4 6h16M4 12h16m-7 6h7"
                ></path>
              </svg>
              <svg
                id="toggle-menu-close"
                aria-hidden="true"
                xmlns="http://www.w3.org/2000/svg"
                width="24"
                height="24"
                class="h-6 w-6 hidden"
                fill="none"
                viewBox="0 0 24 24"
                stroke="currentColor"
                stroke-width="2"
              >
                <path
                  stroke-linecap="round"
                  stroke-linejoin="round"
                  d="M6 18L18 6M6 6l12 12"
                ></path>
              </svg>
            </button>
          </div>
        </div>

        {/* <!-- link section --> */}
        <nav
          id="navigation-contents"
          aria-label="Primary"
          role="navigation"
          class="bg-transparent -translate-y-full w-full md:w-auto md:translate-y-0 md:ml-auto md:mr-[8.3333335%] 2xl:mr-[10%] md:bg-transparent opacity-0 md:opacity-100 transition-all duration-500"
        >
          <div class="mx-auto w-10/12 2xl:w-[80%] md:w-auto">
            <div class="items-center space-x-8 flex py-1 md:py-0 flex-row">
              {/* <!-- primary navigation --> */}
              <div class="flex items-center space-x-8">
                <a
                  class="text-xxs md:text-xs uppercase py-5 hover:underline focus:underline"
                  href="/#about"
                >
                  About
                </a>
                <a
                  class="text-xxs md:text-xs uppercase py-5 hover:underline focus:underline"
                  href="/#work"
                >
                  Work
                </a>
                <a
                  class="text-xxs md:text-xs uppercase py-5 hover:underline focus:underline"
                  href="/#contact"
                >
                  Contact
                </a>
              </div>

              {/* <!-- secondary navigation --> */}
              <div class="flex items-center space-x-1">
                <a
                  href="https://github.lokkeestudios.com"
                  rel="noreferrer"
                  target="_blank"
                  title="GitHub profile"
                  class="w-7 h-7 flex items-center justify-center"
                >
                  <svg
                    xmlns="http://www.w3.org/2000/svg"
                    width="24"
                    height="24"
                    class="w-6 h-6 hover:w-7 hover:h-7 transition-all duration-300"
                    viewBox="0 0 24 24"
                    fill="currentColor"
                  >
                    <path
                      fill-rule="evenodd"
                      clip-rule="evenodd"
                      d="M12.026 2c-5.509 0-9.974 4.465-9.974 9.974 0 4.406 2.857 8.145 6.821 9.465.499.09.679-.217.679-.481 0-.237-.008-.865-.011-1.696-2.775.602-3.361-1.338-3.361-1.338-.452-1.152-1.107-1.459-1.107-1.459-.905-.619.069-.605.069-.605 1.002.07 1.527 1.028 1.527 1.028.89 1.524 2.336 1.084 2.902.829.091-.645.351-1.085.635-1.334-2.214-.251-4.542-1.107-4.542-4.93 0-1.087.389-1.979 1.024-2.675-.101-.253-.446-1.268.099-2.64 0 0 .837-.269 2.742 1.021a9.582 9.582 0 0 1 2.496-.336 9.554 9.554 0 0 1 2.496.336c1.906-1.291 2.742-1.021 2.742-1.021.545 1.372.203 2.387.099 2.64.64.696 1.024 1.587 1.024 2.675 0 3.833-2.33 4.675-4.552 4.922.355.308.675.916.675 1.846 0 1.334-.012 2.41-.012 2.737 0 .267.178.577.687.479C19.146 20.115 22 16.379 22 11.974 22 6.465 17.535 2 12.026 2z"
                    ></path>
                  </svg>
                </a>
                <a
                  href="https://discord.lokkeestudios.com"
                  rel="noreferrer"
                  target="_blank"
                  title="Discord server"
                  class="w-7 h-7 flex items-center justify-center"
                >
                  <svg
                    xmlns="http://www.w3.org/2000/svg"
                    width="24"
                    height="24"
                    class="w-6 h-6 hover:w-7 hover:h-7 transition-all duration-300"
                    viewBox="0 0 24 24"
                    fill="currentColor"
                  >
                    <path d="M14.82 4.26a10.14 10.14 0 0 0-.53 1.1 14.66 14.66 0 0 0-4.58 0 10.14 10.14 0 0 0-.53-1.1 16 16 0 0 0-4.13 1.3 17.33 17.33 0 0 0-3 11.59 16.6 16.6 0 0 0 5.07 2.59A12.89 12.89 0 0 0 8.23 18a9.65 9.65 0 0 1-1.71-.83 3.39 3.39 0 0 0 .42-.33 11.66 11.66 0 0 0 10.12 0q.21.18.42.33a10.84 10.84 0 0 1-1.71.84 12.41 12.41 0 0 0 1.08 1.78 16.44 16.44 0 0 0 5.06-2.59 17.22 17.22 0 0 0-3-11.59 16.09 16.09 0 0 0-4.09-1.35zM8.68 14.81a1.94 1.94 0 0 1-1.8-2 1.93 1.93 0 0 1 1.8-2 1.93 1.93 0 0 1 1.8 2 1.93 1.93 0 0 1-1.8 2zm6.64 0a1.94 1.94 0 0 1-1.8-2 1.93 1.93 0 0 1 1.8-2 1.92 1.92 0 0 1 1.8 2 1.92 1.92 0 0 1-1.8 2z"></path>
                  </svg>
                </a>
              </div>
            </div>
          </div>
        </nav>
      </div>
    </header>
  );
};

const Hero = () => {
  return (
    // Has a bg image
    <section
      id="home"
      class="h-screen w-full bg-brand bg-center bg-no-repeat bg-[length:105%_105%] flex py-28 justify-center items-center transition-all duration-75 sticky top-0"
      aria-label="Hero"
    >
      <div class="mx-auto w-10/12 2xl:w-[80%]">
        <h1 class="font-display font-bold text-5xl leading-tight md:text-7xl md:leading-tight lg:text-8xl lg:leading-tight text-center">
          Software and Hardware Engineer
        </h1>

        <a
          href="/#work"
          class="inline-block px-4 py-3 lg:px-5 lg:py-4 border-2 border-neutrals-50 bg-[length:140%] bg-no-repeat bg-[position:400%] hover:bg-[position:50%] hover:text-neutrals-900 transition-all duration-500 lg:text-lg mt-16 relative left-1/2 -translate-x-1/2"
          //   style="background-image: url('/assets/button-light.svg')"
        >
          Dig into my universe
        </a>
      </div>
    </section>
  );
};

const About = () => {
  return (
    <section
      id="about"
      class="w-full flex items-center bg-neutral-900 py-28 relative z-10 xl:min-h-screen flex-col md:flex-row"
      aria-label="About"
    >
      <img
        class="object-cover object-center w-full max-h-screen relative -top-28 md:top-0 md:w-1/2 md:h-full md:pr-10 xl:pr-0 md:absolute"
        loading="lazy"
        width="756"
        height="1008"
        src="/assets/about.webp"
        alt="A picture of myself, lokkee"
      />
      <div class="mx-auto w-10/12 2xl:w-[80%] h-full">
        <div class="flex flex-col md:flex-row h-full w-full">
          <div class="basis-1/2 md:self-center ml-auto xl:pl-20">
            <h2 class="font-display font-bold text-brand lg:text-xl uppercase mb-4 text-yellow-400">
              About
            </h2>

            <div class="inline-block">
              <h3
                id="typewriter"
                class="font-bold text-4xl lg:text-5xl mb-8 font-mono overflow-hidden whitespace-nowrap w-0"
              >
                Error (╯°□°)╯︵ ┻━┻
              </h3>
            </div>
            <p class="text-neutral-50/60 leading-relaxed max-w-prose">
              I'm Lokkee, a passionate freelancer bringing you programming and
              design from the future. I am experienced in developing web and
              desktop applications including full front end design. This
              includes brand identity, graphics and illustrations.
            </p>
          </div>
        </div>
      </div>
    </section>
  );
};
