<script>
  /** @typedef {'Blink' | 'Fade' | 'Breathe' | 'Pockets'} EffectTab */

  /** @type {EffectTab} */
  let activeTab = 'Blink';

  /** @type {EffectTab[]} */
  const effects = ['Blink', 'Fade', 'Breathe', 'Pockets'];

  // Blink RGB color ratios
  let blinkRed = 255;
  let blinkGreen = 0;
  let blinkBlue = 0;

  // Brightness percentage and on/off interval
  let blinkBrightness = 100;
  let blinkInterval = 500;

  // Calculated RGB values sent to the LED driver
  let blinkOutputRed = 0;
  let blinkOutputGreen = 0;
  let blinkOutputBlue = 0;

  // Scale RGB ratios using the firmware's configured brightness limit
  $: blinkOutputRed = Math.round(
    (blinkRed / 255) * (blinkBrightness / 100) * __LED_MAX_BRIGHTNESS__
  );

  $: blinkOutputGreen = Math.round(
    (blinkGreen / 255) * (blinkBrightness / 100) * __LED_MAX_BRIGHTNESS__
  );

  $: blinkOutputBlue = Math.round(
    (blinkBlue / 255) * (blinkBrightness / 100) * __LED_MAX_BRIGHTNESS__
  );

  // Browser preview uses the full RGB range
  $: blinkPreviewRed = Math.round(blinkRed * blinkBrightness / 100);
  $: blinkPreviewGreen = Math.round(blinkGreen * blinkBrightness / 100);
  $: blinkPreviewBlue = Math.round(blinkBlue * blinkBrightness / 100);

</script>
<main class="min-h-screen flex items-center justify-center bg-slate-950 px-4">
  <div class="w-full max-w-md text-center">
    <h1 class="text-5xl font-bold text-white">LED Effects</h1>
    <p class="mt-4 text-lg text-slate-400">
      Select and configure LED cube effects.
    </p>
    
    <div class="mt-10 grid grid-cols-4 gap-2">
      {#each effects as effect}
          <button
            type="button"
            onclick={() => activeTab = effect}
            class={`rounded-lg px-3 py-3 text-sm font-semibold text-white transition-colors ${
              activeTab === effect
                  ? 'bg-blue-600'
                  : 'bg-slate-800 hover:bg-slate-700'
            }`}
          >
            {effect}
          </button>
      {/each}
  </div>
  <!-- Configuration panel for selected effect -->
  <div class="mt-6 rounded-lg bg-slate-900 p-6 text-left">
    <h2 class="text-xl font-semibold text-white">
        {activeTab} Configuration
    </h2>

    <p class="mt-2 text-sm text-slate-400">
        Configure the {activeTab.toLowerCase()} effect.
    </p>
    {#if activeTab === 'Blink'}

      <!-- Live preview of the calculated LED color -->
      <div class="mt-6">
        <div class="flex items-center justify-between">
            <span class="text-sm font-medium text-slate-300">
              Color Preview
            </span>
            <span class="text-xs text-slate-400">
              RGB ({blinkOutputRed}, {blinkOutputGreen}, {blinkOutputBlue})
            </span>
        </div>

        <div
            class="mt-2 h-14 w-full rounded-lg border border-slate-700"
            style={`background-color: rgb(${blinkPreviewRed}, ${blinkPreviewGreen}, ${blinkPreviewBlue})`}
        ></div>
      </div>

      <!-- Red channel ratio -->
      <div class="mt-6">
        <label for="blink-red" class="block text-sm font-medium text-slate-300">
            Red: {blinkRed}
        </label>

        <input
            id="blink-red"
            type="range"
            min="0"
            max="255"
            step="1"
            bind:value={blinkRed}
            class="mt-3 w-full cursor-pointer accent-red-500"
        />
      </div>

      <!-- Green channel ratio -->
      <div class="mt-6">
        <label for="blink-green" class="block text-sm font-medium text-slate-300">
            Green: {blinkGreen}
        </label>

        <input
            id="blink-green"
            type="range"
            min="0"
            max="255"
            step="1"
            bind:value={blinkGreen}
            class="mt-3 w-full cursor-pointer accent-green-500"
        />
      </div>

      <!-- Blue channel ratio -->
      <div class="mt-6">
        <label for="blink-blue" class="block text-sm font-medium text-slate-300">
            Blue: {blinkBlue}
        </label>

        <input
            id="blink-blue"
            type="range"
            min="0"
            max="255"
            step="1"
            bind:value={blinkBlue}
            class="mt-3 w-full cursor-pointer accent-blue-500"
        />
      </div>

      <!-- Brightness as a percentage of the configured maximum -->
      <div class="mt-6">
        <label for="blink-brightness" class="block text-sm font-medium text-slate-300">
            Brightness: {blinkBrightness}%
        </label>

        <input
            id="blink-brightness"
            type="range"
            min="0"
            max="100"
            step="1"
            bind:value={blinkBrightness}
            class="mt-3 w-full cursor-pointer accent-blue-600"
        />
      </div>
      <div class="mt-6">
        <label for="blink-interval" class="block text-sm font-medium text-slate-300">
            Blink Interval: {blinkInterval} ms
        </label>

        <input
            id="blink-interval"
            type="range"
            min="100"
            max="2000"
            step="50"
            bind:value={blinkInterval}
            class="mt-3 w-full cursor-pointer accent-blue-600"
        />
      </div>
      <button
        type="button"
        class="mt-8 w-full rounded-lg bg-blue-600 px-6 py-3 font-semibold text-white hover:bg-blue-500"
      >
        Activate Blink
      </button>
    {/if}
  </div>
  </div>
</main>