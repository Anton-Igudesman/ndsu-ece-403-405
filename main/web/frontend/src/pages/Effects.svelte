<script>
  /** @typedef {'Blink' | 'Fade' | 'Breathe' | 'Pockets'} EffectTab */

  /** @type {EffectTab} */
  let activeTab = 'Blink';

  /** @type {EffectTab[]} */
  const effects = ['Blink', 'Fade', 'Breathe', 'Pockets'];

  // Blink effect configuration
  let blinkColor = '#ff0000';
  let blinkInterval = 500;

</script>
<main class="min-h-screen flex items-center justify-center bg-slate-950 px-4">
  <div class="w-full max-w-md text-center">
    <h1 class="text-5xl font-bold text-white">LED Effects</h1>
    <p class="mt-4 text-lg text-slate-400">
      Select and configure LED cube effects.
    </p>
    {#if activeTab === 'Blink'}
      <div class="mt-6">
          <label for="blink-color" class="block text-sm font-medium text-slate-300">
            Blink Color
          </label>

          <input
            id="blink-color"
            type="color"
            bind:value={blinkColor}
            class="mt-2 h-10 w-full cursor-pointer rounded-lg bg-slate-800"
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
  </div>
  </div>
</main>