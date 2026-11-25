<script>
  import { onMount } from 'svelte';

  /** @typedef {'Blink' | 'Fade' | 'Breathe' | 'Pockets' | 'Text'} EffectTab */

  /** @type {EffectTab} */
  let activeTab = 'Blink';

  /** @type {EffectTab[]} */
  const effects = ['Blink', 'Fade', 'Breathe', 'Pockets', 'Text'];

  // Effects Dashboard WebSocket connection and ownership state
  /** @type {WebSocket | null} */
  let effectsSocket = null;

  let effectsConnected = false;
  let isEffectsOwner = false;
  let effectsOwnerPresent = false;
  let isEffectsAdmin = false;

  // Track whether this client has requested LED control
  let effectsControlPending = false;

  // Feedback from controller ownership operations
  let effectsControlStatus = '';

  // Current LED operating mode reported by the ESP32
  let activeLedMode = 'unknown';

  // Track whether the ESP32 is transitioning between LED modes
  let effectsTransitioning = false;

  // Feedback from ESP32 after saving a layer
  let blinkSaveStatus = '';

  // Track whether a Blink configuration save is awaiting acknowledgment
  let blinkSavePending = false;

  // Identify the layer awaiting a Save acknowledgment
  /** @type {number | null} */
  let blinkPendingLayer = null;

  // Prevent overlapping Blink mode requests
  let blinkModePending = false;

  // Track which Blink layers have loaded their configurations from the ESP32
  /** @type {boolean[]} */
  let blinkLayersLoaded = Array(8).fill(false);

  /**
   * @typedef {'standard' | 'weave' | 'moving_face'} TextAnimationStyle
   */

  /** @type {TextAnimationStyle} */
  let textAnimationStyle = 'standard';

  // Current text configuration loaded from ESP32
  let textMessage = '';
  let textDefaultColor = '#000065';
  let textCharacterColors = Array(12).fill('#000065');
  let textScrollInterval = 150;

  // Track firmware configuration and command acknowledgments
  let textConfigLoaded = false;
  let textSavePending = false;
  let textModePending = false;
  let textStatus = '';

  // Future animation settings (not yet supported by firmware)
  let textDepthEnabled = false;
  let textDepthSpeed = 150;
  let textDepthMin = 0;
  let textDepthMax = 7;

  // Connect to the ESP32's existing terminal WebSocket
  function connectEffectsSocket() {
    // Determine whether this device can take control as an administrator
    async function refreshEffectsAdminStatus() {
      try {
        const response = await fetch('/api/controller/status', {
          cache: 'no-store'
        });

        if (!response.ok) {
          isEffectsAdmin = false;
          return;
        }

        const data = await response.json();
        isEffectsAdmin = data.auth_state === 3;
      } catch (error) {
        console.error('Unable to retrieve Effects administrator status:', error);
        isEffectsAdmin = false;
      }
    }
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const url = `${protocol}//${window.location.host}/ws/terminal`;

    effectsSocket = new WebSocket(url);

    effectsSocket.onopen = () => {
      // Require fresh configuration readback for this connection
      refreshEffectsAdminStatus();
      blinkLayersLoaded = Array(8).fill(false);
      effectsConnected = true;

      // Retrieve current controller mode and ownership status
      effectsSocket?.send(JSON.stringify({ action: 'get_mode' }));
      effectsSocket?.send(JSON.stringify({ action: 'get_ownership' }));

      // Retrieve the saved configuration for all eight physical layers
      for (const layer of blinkLayers) {
        effectsSocket?.send(JSON.stringify({
          action: 'blink_get_layer',
          layer
        }));
      }

      // Retrieve the saved Fade configuration for all eight physical layers
      for (const layer of blinkLayers) {
        effectsSocket?.send(JSON.stringify({
          action: 'fade_get_layer',
          layer
        }));
      }

      // Retrieve saved scrolling text configuration
      textConfigLoaded = false;
      effectsSocket?.send(JSON.stringify({ action: 'text_get_config' }));
    };

    // Process controller mode and ownership updates
    effectsSocket.onmessage = (event) => {
      try {
        const message = JSON.parse(event.data);

        if (message.event === 'terminal_ownership_changed' ||
            message.event === 'terminal_ownership_status_result') {
              isEffectsOwner = message.is_owner === true;
              effectsOwnerPresent = message.owner_present === true;
              effectsControlPending = message.request_pending === true;
            }

        // Handle control request and release acknowledgments
        if (message.event === 'control_request_result' ||
            message.event === 'control_release_result' ||
            message.event === 'control_takeover_result') {
          if (message.success !== true) {
            console.error('LED control operation failed:', message.error);
            effectsControlStatus = `Control operation failed: ${message.error ?? 'Unknown error'}`;
          } else effectsControlStatus = '';

          // Refresh ownership state after the operation
          effectsSocket?.send(JSON.stringify({ action: 'get_ownership' }));
        }

        // Track the active LED mode and ongoing transitions
        if (message.event === 'mode_status' && message.success === true) {
          activeLedMode = message.mode;
          effectsTransitioning = message.transitioning === true;
          if (!effectsTransitioning) blinkModePending = false;
        }

        // Process Blink activation and deactivation acknowledgments
        if (message.event === 'blink_activation_result' ||
            message.event === 'blink_deactivation_result') {
          if (message.success !== true) {
            blinkModePending = false;
            console.error('Blink mode change failed:', message.error);
          }

          // Refresh controller state after the request
          effectsSocket?.send(JSON.stringify({ action: 'get_mode' }));
        }

        // Process the ESP32's Blink configuration save result
        if (message.event === 'blink_save_result') {
          const savedLayer = blinkPendingLayer;
          blinkSavePending = false;
          blinkPendingLayer = null;

          if (message.success === true && message.layer === savedLayer) {
            blinkSaveStatus = `Layer ${savedLayer} settings saved successfully`;
          } else {
            blinkSaveStatus = `Save failed: ${message.error ?? 'Unexpected layer acknowledgment'}`;
            console.error('Blink save failed:', message);
          }
        }

        // Process the ESP32's Fade configuration save result
        if (message.event === 'fade_save_result') {
          const savedLayer = fadePendingLayer;

          fadeSavePending = false;
          fadePendingLayer = null;

          if (message.success === true && message.layer === savedLayer) {
            fadeSaveStatus = `Layer ${savedLayer} settings saved successfully`;
          } else {
            fadeSaveStatus = `Save failed: ${message.error ?? 'Unexpected layer acknowledgment'}`;
            console.error('Fade save failed:', message);
          }
        }

        // Process Fade activation and deactivation acknowledgments
        if (message.event === 'fade_activation_result' ||
            message.event === 'fade_deactivation_result') {
          if (message.success !== true) {
            console.error('Fade mode change failed:', message.error);
          }

          // Refresh the controller's actual operating mode
          effectsSocket?.send(JSON.stringify({ action: 'get_mode' }));
        }

        // Process Text activation and deactivation acknowledgments
        if (message.event === 'text_activation_result' ||
            message.event === 'text_deactivation_result') {
          textModePending = false;

          if (message.success !== true) {
            textStatus = `Text mode change failed: ${message.error ?? 'Unknown error'}`;
          } else {
            textStatus = '';
          }

          // Refresh the controller's actual operating mode
          effectsSocket?.send(JSON.stringify({ action: 'get_mode' }));
        }

        // Handle scrolling text configuration command results
        if ([
          'text_message_result',
          'text_default_color_result',
          'text_character_color_result',
          'text_reset_colors_result',
          'text_scroll_interval_result',
          'text_reset_config_result'
        ].includes(message.event)) {
          textSavePending = false;

          if (message.success !== true) {
            textStatus = `Text update failed: ${message.error ?? 'Unknown error'}`;
          } else {
            textStatus = 'Text settings updated successfully';

            // Read back the configuration confirmed by the ESP32
            effectsSocket?.send(JSON.stringify({ action: 'text_get_config' }));
          }
        }

        // Load saved Fade configurations returned by the ESP32
        if (message.event === 'fade_layer_config' && message.success === true) {
          const layer = message.layer;

          if (Number.isInteger(layer) && layer >= 0 && layer < fadeConfigs.length) {
            fadeConfigs[layer] = {
              colors: [
                rgbToHex(message.color1),
                rgbToHex(message.color2),
                rgbToHex(message.color3)
              ],
              colorCount: message.color_count,
              brightness: message.brightness,
              transitionSteps: message.transition_steps
            };

            // Notify Svelte that the selected layer configuration changed
            fadeConfigs = [...fadeConfigs];
          }
        }

        // Load saved Blink configurations returned by the ESP32
        if (message.event === 'blink_layer_config' && message.success === true) {
          const layer = message.layer;

          if (Number.isInteger(layer) && layer >= 0 && layer < blinkConfigs.length) {
            blinkConfigs[layer] = {
              red: message.r,
              green: message.g,
              blue: message.b,
              brightness: message.brightness,
              interval: message.interval_ms
            };

            // Mark this layer ready after receiving its firmware configuration
            blinkLayersLoaded[layer] = true;
            blinkLayersLoaded = [...blinkLayersLoaded];

            // Update sliders only if this is the selected layer
            if (layer === activeBlinkLayer) {
              blinkRed = message.r;
              blinkGreen = message.g;
              blinkBlue = message.b;
              blinkBrightness = message.brightness;
              blinkInterval = message.interval_ms;
            }
          }
        }

        // Load scrolling text settings returned by the ESP32
        if (message.event === 'text_config' && message.success === true) {
          textMessage = message.message;
          textDefaultColor = rgbToHex(message.default_color);
          textCharacterColors = message.character_colors.map(rgbToHex);
          textScrollInterval = message.interval_ms;
          textConfigLoaded = true;
        }
      } catch (error) {
        console.error('Invalid Effects WebSocket message:', error);
      }
    };

    effectsSocket.onclose = () => {
      effectsConnected = false;
      isEffectsOwner = false;
      effectsOwnerPresent = false;
      effectsControlPending = false;
      effectsTransitioning = false;
      activeLedMode = 'unknown';

      // Discard pending operations and stale firmware configuration state
      blinkSavePending = false;
      blinkPendingLayer = null;
      blinkModePending = false;
      blinkLayersLoaded = Array(8).fill(false);
      blinkSaveStatus = '';
      fadeSavePending = false;
      fadePendingLayer = null;
      fadeSaveStatus = '';
      effectsControlStatus = '';

      // Discard pending Text operations and require fresh configuration
      textConfigLoaded = false;
      textSavePending = false;
      textModePending = false;
      textStatus = '';
    };
  }

  // Open the Effects WebSocket when the page loads and close it when leaving
  onMount(() => {
    connectEffectsSocket();

    // Keep the dashboard synchronized with ESP32 mode changes
    const modePoll = window.setInterval(() => {
      if (effectsSocket?.readyState === WebSocket.OPEN) {
        effectsSocket.send(JSON.stringify({ action: 'get_mode' }));
      }
    }, 500);

    return () => {
      window.clearInterval(modePoll);
      effectsSocket?.close();
      effectsSocket = null;
    };
  });

  // Physical LED cube layer currently being configured
  let activeBlinkLayer = 0;

  // Available physical layers (Z = 0 through 7)
  const blinkLayers = Array.from({ length: 8 }, (_, index) => index);

  /**
   * Configuration for one physical Blink layer.
   * @typedef {Object} BlinkLayerConfig
   * @property {number} red - RGB red channel (0-255)
   * @property {number} green - RGB green channel (0-255)
   * @property {number} blue - RGB blue channel (0-255)
   * @property {number} brightness - Brightness percentage (0-100)
   * @property {number} interval - ON/OFF duration in milliseconds
   */

  /**
   * Configuration for one physical Fade layer.
   * @typedef {Object} FadeLayerConfig
   * @property {string[]} colors - Three RGB colors stored as hexadecimal strings
   * @property {number} colorCount - Number of active colors (2 or 3)
   * @property {number} brightness - Brightness percentage (0-100)
   * @property {number} transitionSteps - Number of interpolation steps per color
   */

  // Independent settings for each physical Blink layer
  /** @type {BlinkLayerConfig[]} */
  let blinkConfigs = Array.from({ length: 8 }, () => ({
    red: 255,
    green: 0,
    blue: 0,
    brightness: 100,
    interval: 500
  }));

  // Independent Fade settings for all eight physical layers
  /** @type {FadeLayerConfig[]} */
  let fadeConfigs = Array.from({ length: 8 }, () => ({
    colors: ['#ff0000', '#00ff00', '#0000ff'],
    colorCount: 3,
    brightness: 100,
    transitionSteps: 200
  }));

  // Physical Fade layer currently being configured
  let activeFadeLayer = 0;

  // Track Fade configuration saves and their acknowledgments
  let fadeSavePending = false;

  /** @type {number | null} */
  let fadePendingLayer = null;

  let fadeSaveStatus = '';
  
  // Load the selected layer's configuration into the sliders
  /** @param {number} layer */
  function selectBlinkLayer(layer) {

    const config = blinkConfigs[layer];

    if (!config) return;

    // Preserve the current layer's edits before switching
    blinkConfigs[activeBlinkLayer] = {
      red: blinkRed,
      green: blinkGreen,
      blue: blinkBlue,
      brightness: blinkBrightness,
      interval: blinkInterval
    };

    activeBlinkLayer = layer;
    blinkRed = config.red;
    blinkGreen = config.green;
    blinkBlue = config.blue;
    blinkBrightness = config.brightness;
    blinkInterval = config.interval;
  }

  // Capture the selected layer's current slider settings
  function getCurrentBlinkConfig() {
    return {
      red: blinkRed,
      green: blinkGreen,
      blue: blinkBlue,
      brightness: blinkBrightness,
      interval: blinkInterval
    };
  }

  // Send the selected Blink layer configuration to the ESP32
  function saveBlinkLayer() {
    if (!effectsSocket || effectsSocket.readyState !== WebSocket.OPEN)
      return;

    if (!isEffectsOwner)
      return;

    // Prevent another Save while awaiting the ESP32's response
    if (blinkSavePending)
      return;

    const config = getCurrentBlinkConfig();

    // Mark the Save operation as pending before transmitting
    blinkSavePending = true;
    blinkPendingLayer = activeBlinkLayer;
    blinkSaveStatus = '';

    effectsSocket.send(JSON.stringify({
      action: 'blink_save_layer',
      layer: activeBlinkLayer,
      r: config.red,
      g: config.green,
      b: config.blue,
      brightness: config.brightness,
      interval_ms: config.interval
    }));
  }

  /**
   * Convert a hexadecimal color into RGB channels.
   * @param {string} hex
   * @returns {{ r: number, g: number, b: number }}
   */
  function hexToRgb(hex) {
    return {
      r: parseInt(hex.slice(1, 3), 16),
      g: parseInt(hex.slice(3, 5), 16),
      b: parseInt(hex.slice(5, 7), 16)
    };
  }

  /**
   * Convert RGB channels from the ESP32 into a color-picker value.
   * @param {{ r: number, g: number, b: number }} color
   * @returns {string}
   */
  function rgbToHex(color) {
    return '#' + [color.r, color.g, color.b]
      .map(channel => channel.toString(16).padStart(2, '0'))
      .join('');
  }

  // Save the selected Fade layer configuration to the ESP32
  function saveFadeLayer() {
    if (!effectsSocket || effectsSocket.readyState !== WebSocket.OPEN)
      return;

    if (!isEffectsOwner || fadeSavePending)
      return;

    const config = fadeConfigs[activeFadeLayer];
    if (!config) return;

    // Convert all three colors, including an inactive third color
    const colors = config.colors.map(hexToRgb);

    fadeSavePending = true;
    fadePendingLayer = activeFadeLayer;
    fadeSaveStatus = '';

    effectsSocket.send(JSON.stringify({
      action: 'fade_save_layer',
      layer: activeFadeLayer,
      color_count: config.colorCount,
      color1: colors[0],
      color2: colors[1],
      color3: colors[2],
      brightness: config.brightness,
      transition_steps: config.transitionSteps
    }));
  }

  // Activate or deactivate Blink based on the ESP32's reported mode
  function toggleBlink() {
    if (!effectsSocket || effectsSocket.readyState !== WebSocket.OPEN)
      return;

    if (!isEffectsOwner || effectsTransitioning || blinkModePending)
      return;

    const action = activeLedMode === 'blink'
      ? 'blink_deactivate'
      : 'blink_activate';

    blinkModePending = true;

    effectsSocket.send(JSON.stringify({ action }));
  }

  // Activate or deactivate Fade using the existing LED controller
  function toggleFade() {
    if (!effectsSocket || effectsSocket.readyState !== WebSocket.OPEN)
      return;

    if (!isEffectsOwner || effectsTransitioning)
      return;

    const action = activeLedMode === 'fade'
      ? 'fade_deactivate'
      : 'fade_activate';

    effectsSocket.send(JSON.stringify({ action }));
  }

  /**
   * Send one Text configuration command to the ESP32.
   * @param {string} action
   * @param {Record<string, unknown>} [params]
   */
  function sendTextCommand(action, params = {}) {
    if (!effectsSocket || effectsSocket.readyState !== WebSocket.OPEN) return;
    if (!isEffectsOwner || !textConfigLoaded || textSavePending) return;

    textSavePending = true;
    textStatus = '';

    effectsSocket.send(JSON.stringify({ action, ...params }));
  }

  // Save the scrolling message
  function saveTextMessage() {
    sendTextCommand('text_set_message', { message: textMessage });
  }

  // Save the default RGB color
  function saveTextDefaultColor() {
    sendTextCommand('text_set_default_color', {
      color: hexToRgb(textDefaultColor)
    });
  }

  // Save one character's RGB color
  /** @param {number} index */
  function saveTextCharacterColor(index) {
    sendTextCommand('text_set_character_color', {
      index,
      color: hexToRgb(textCharacterColors[index])
    });
  }

  // Save the scrolling interval
  function saveTextScrollInterval() {
    sendTextCommand('text_set_scroll_interval', {
      interval_ms: textScrollInterval
    });
  }

  // Restore character colors to their defaults
  function resetTextColors() {
    sendTextCommand('text_reset_colors');
  }

  // Restore the complete text configuration
  function resetTextConfig() {
    sendTextCommand('text_reset_config');
  }

  // Activate or deactivate Text through the existing LED controller
  function toggleText() {
    if (!effectsSocket || effectsSocket.readyState !== WebSocket.OPEN) return;
    if (!isEffectsOwner || !textConfigLoaded ||
        effectsTransitioning || textModePending) return;

    const action = activeLedMode === 'text'
      ? 'text_deactivate'
      : 'text_activate';

    textModePending = true;
    textStatus = '';

    effectsSocket.send(JSON.stringify({ action }));
  }

  // Request or release LED control through the existing Terminal WebSocket
  function toggleEffectsControl() {
    if (!effectsSocket || effectsSocket.readyState !== WebSocket.OPEN)
      return;

    if (effectsControlPending)
      return;

    const action = isEffectsOwner
      ? 'release_control'
      : isEffectsAdmin
        ? 'admin_takeover'
        : 'request_control';

    effectsSocket.send(JSON.stringify({ action }));
  }

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
    
    <!-- Request or release control of the LED cube -->
    <button
      type="button"
      onclick={toggleEffectsControl}
      disabled={!effectsConnected || effectsControlPending}
      class="mt-6 w-full rounded-lg bg-slate-700 px-6 py-3 font-semibold text-white hover:bg-slate-600 disabled:cursor-not-allowed disabled:opacity-50"
    >
      {isEffectsOwner
        ? 'Release Control'
        : effectsControlPending
          ? 'Control Request Pending'
          : isEffectsAdmin
            ? 'Take Control'
            : 'Request Control'}
    </button>

    <!-- Display controller ownership errors -->
    {#if effectsControlStatus}
      <p class="mt-3 text-center text-sm text-red-400">
        {effectsControlStatus}
      </p>
    {/if}
    <div class="mt-10 grid grid-cols-5 gap-2">
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

      <!-- Select the physical Blink layer to configure -->
      <div class="mt-6 grid grid-cols-4 gap-2">
        {#each blinkLayers as layer}
            <button
              type="button"
              onclick={() => selectBlinkLayer(layer)}
              class={`rounded-lg px-3 py-2 text-sm font-semibold text-white transition-colors ${
                  activeBlinkLayer === layer
                    ? 'bg-blue-600'
                    : 'bg-slate-800 hover:bg-slate-700'
              }`}
            >
              Layer {layer}
            </button>
        {/each}
      </div>

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
            disabled={!blinkLayersLoaded[activeBlinkLayer]}
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
            disabled={!blinkLayersLoaded[activeBlinkLayer]}
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
            disabled={!blinkLayersLoaded[activeBlinkLayer]}
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
            disabled={!blinkLayersLoaded[activeBlinkLayer]}
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
            disabled={!blinkLayersLoaded[activeBlinkLayer]}
            type="range"
            min="200"
            max="1500"
            step="50"
            bind:value={blinkInterval}
            class="mt-3 w-full cursor-pointer accent-blue-600"
        />
      </div>

      <!-- Save the selected layer's configuration -->
      <button
        type="button"
        disabled={!effectsConnected || 
                  !isEffectsOwner || 
                  !blinkLayersLoaded[activeBlinkLayer] || 
                  blinkSavePending}
        onclick={saveBlinkLayer}
        class="mt-8 w-full rounded-lg bg-green-600 px-6 py-3 font-semibold text-white hover:bg-green-500 disabled:cursor-not-allowed disabled:opacity-50"
      >
        Save Layer {activeBlinkLayer}
      </button>

      <!-- Display the ESP32's configuration save result -->
      {#if blinkSaveStatus}
        <p class="mt-3 text-center text-sm text-slate-300">
          {blinkSaveStatus}
        </p>
      {/if}

      <button
        type="button"
        onclick={toggleBlink}
        disabled={!effectsConnected || 
                !isEffectsOwner || 
                activeLedMode === 'unknown' || 
                effectsTransitioning ||
                blinkModePending}
        class="mt-8 w-full rounded-lg bg-blue-600 px-6 py-3 font-semibold text-white hover:bg-blue-500 disabled:cursor-not-allowed disabled:opacity-50"
      >
        {activeLedMode === 'blink' ? 'Deactivate Blink' : 'Activate Blink'}
      </button>
    {:else if activeTab === 'Fade'}

    <!-- Select the physical Fade layer to configure -->
    <div class="mt-6 grid grid-cols-4 gap-2">
      {#each blinkLayers as layer}
        <button
          type="button"
          onclick={() => activeFadeLayer = layer}
          class={`rounded-lg px-3 py-2 text-sm font-semibold text-white transition-colors ${
            activeFadeLayer === layer
              ? 'bg-blue-600'
              : 'bg-slate-800 hover:bg-slate-700'
          }`}
        >
          Layer {layer}
        </button>
      {/each}
    </div>
    <!-- Select how many colors participate in the Fade sequence -->
    <div class="mt-6">
      <p class="text-sm font-medium text-slate-300">
        Number of Colors
      </p>

      <div class="mt-3 flex gap-6">
        <label class="flex cursor-pointer items-center gap-2 text-sm text-slate-300">
          <input
            type="radio"
            name="fade-color-count"
            value="2"
            checked={fadeConfigs[activeFadeLayer].colorCount === 2}
            onchange={() => fadeConfigs[activeFadeLayer].colorCount = 2}
            class="accent-blue-600"
          />
          2 Colors
        </label>

        <label class="flex cursor-pointer items-center gap-2 text-sm text-slate-300">
          <input
            type="radio"
            name="fade-color-count"
            value="3"
            checked={fadeConfigs[activeFadeLayer].colorCount === 3}
            onchange={() => fadeConfigs[activeFadeLayer].colorCount = 3}
            class="accent-blue-600"
          />
          3 Colors
        </label>
      </div>
      <!-- Configure the selected layer's Fade colors -->
      <div class="mt-6 grid grid-cols-3 gap-3">
        {#each fadeConfigs[activeFadeLayer].colors.slice(0, fadeConfigs[activeFadeLayer].colorCount) as color, index}
          <div>
            <label
              for={`fade-color-${index}`}
              class="mb-2 block text-center text-sm font-medium text-slate-300"
            >
              Color {index + 1}
            </label>

            <input
              id={`fade-color-${index}`}
              type="color"
              value={color}
              oninput={(event) => {
                fadeConfigs[activeFadeLayer].colors[index] = event.currentTarget.value;
                fadeConfigs = [...fadeConfigs];
              }}
              class="h-16 w-full cursor-pointer rounded-lg border border-slate-700 bg-transparent"
            />
          </div>
        {/each}
      </div>
    </div>

    <!-- Brightness percentage for the selected Fade layer -->
    <div class="mt-6">
      <label for="fade-brightness" class="block text-sm font-medium text-slate-300">
        Brightness: {fadeConfigs[activeFadeLayer].brightness}%
      </label>

      <input
        id="fade-brightness"
        type="range"
        min="0"
        max="100"
        step="1"
        value={fadeConfigs[activeFadeLayer].brightness}
        oninput={(event) => {
          fadeConfigs[activeFadeLayer].brightness = Number(event.currentTarget.value);
          fadeConfigs = [...fadeConfigs];
        }}
        class="mt-3 w-full cursor-pointer accent-blue-600"
      />
    </div>

    <!-- Number of interpolation steps between consecutive Fade colors -->
    <div class="mt-6">
      <label for="fade-steps" class="block text-sm font-medium text-slate-300">
        Fade Transition: {fadeConfigs[activeFadeLayer].transitionSteps} steps
      </label>

      <p class="mt-1 text-xs text-slate-400">
        Higher values produce slower transitions. The default is 200 steps;
        actual duration depends on the LED controller's update rate.
      </p>

      <input
        id="fade-steps"
        type="range"
        min="100"
        max="500"
        step="50"
        value={fadeConfigs[activeFadeLayer].transitionSteps}
        oninput={(event) => {
          fadeConfigs[activeFadeLayer].transitionSteps = Number(event.currentTarget.value);
          fadeConfigs = [...fadeConfigs];
        }}
        class="mt-3 w-full cursor-pointer accent-blue-600"
      />
    </div>

    <!-- Save the selected Fade layer's configuration -->
    <button
      type="button"
      onclick={saveFadeLayer}
      disabled={!effectsConnected ||
                !isEffectsOwner ||
                fadeSavePending}
      class="mt-8 w-full rounded-lg bg-green-600 px-6 py-3 font-semibold text-white hover:bg-green-500 disabled:cursor-not-allowed disabled:opacity-50"
    >
      Save Layer {activeFadeLayer}
    </button>

    <!-- Display the ESP32's Fade configuration save result -->
    {#if fadeSaveStatus}
      <p class="mt-3 text-center text-sm text-slate-300">
        {fadeSaveStatus}
      </p>
    {/if}

    <!-- Activate or deactivate Fade through the LED controller -->
    <button
      type="button"
      onclick={toggleFade}
      disabled={!effectsConnected ||
                !isEffectsOwner ||
                activeLedMode === 'unknown' ||
                effectsTransitioning}
      class="mt-8 w-full rounded-lg bg-blue-600 px-6 py-3 font-semibold text-white hover:bg-blue-500 disabled:cursor-not-allowed disabled:opacity-50"
    >
      {activeLedMode === 'fade' ? 'Deactivate Fade' : 'Activate Fade'}
    </button>

  {:else if activeTab === 'Text'}
    <!-- Configure the scrolling message -->
    <div class="mt-6">
      <label for="text-message" class="block text-sm font-medium text-slate-300">
        Message ({textMessage.length}/12)
      </label>

      <input
        id="text-message"
        type="text"
        maxlength="12"
        bind:value={textMessage}
        disabled={!textConfigLoaded || !isEffectsOwner}
        class="mt-3 w-full rounded-lg border border-slate-700 bg-slate-800 px-3 py-3 text-white"
      />

      <button
        type="button"
        onclick={saveTextMessage}
        disabled={!effectsConnected || !isEffectsOwner || !textConfigLoaded ||
                  textSavePending || textMessage.length === 0}
        class="mt-3 w-full rounded-lg bg-green-600 px-6 py-3 font-semibold text-white disabled:opacity-50"
      >
        Save Message
      </button>
    </div>

      <!-- Configure the default color for scrolling text -->
      <div class="mt-6">
        <label for="text-default-color" class="block text-sm font-medium text-slate-300">
          Default Text Color
        </label>

        <input
          id="text-default-color"
          type="color"
          bind:value={textDefaultColor}
          disabled={!textConfigLoaded || !isEffectsOwner}
          class="mt-3 h-14 w-full cursor-pointer rounded-lg border border-slate-700 bg-transparent"
        />

        <button
          type="button"
          onclick={saveTextDefaultColor}
          disabled={!effectsConnected || !isEffectsOwner ||
                    !textConfigLoaded || textSavePending}
          class="mt-3 w-full rounded-lg bg-green-600 px-6 py-3 font-semibold text-white disabled:opacity-50"
        >
          Save Default Color
        </button>
      </div>

      <!-- Configure an individual RGB color for each message character -->
      <div class="mt-6">
        <h3 class="text-sm font-medium text-slate-300">
          Character Colors
        </h3>

        <div class="mt-3 grid grid-cols-3 gap-3">
          {#each [...textMessage] as character, index}
            <div>
              <label
                for={`text-character-${index}`}
                class="mb-2 block text-center text-sm text-slate-300"
              >
                {character === ' ' ? 'Space' : character} ({index + 1})
              </label>

              <input
                id={`text-character-${index}`}
                type="color"
                value={textCharacterColors[index] ?? textDefaultColor}
                disabled={!textConfigLoaded || !isEffectsOwner || textSavePending}
                oninput={(event) => {
                  textCharacterColors[index] = event.currentTarget.value;
                  textCharacterColors = [...textCharacterColors];
                }}
                class="h-12 w-full cursor-pointer rounded-lg border border-slate-700 bg-transparent"
              />

              <button
                type="button"
                onclick={() => saveTextCharacterColor(index)}
                disabled={!effectsConnected || !isEffectsOwner ||
                          !textConfigLoaded || textSavePending}
                class="mt-2 w-full rounded-lg bg-green-600 px-2 py-2 text-xs font-semibold text-white disabled:opacity-50"
              >
                Save
              </button>
            </div>
          {/each}
        </div>
      </div>

      <!-- Configure how quickly text advances by one column -->
      <div class="mt-6">
        <label for="text-scroll-interval" class="block text-sm font-medium text-slate-300">
          Scroll Interval: {textScrollInterval} ms
        </label>

        <input
          id="text-scroll-interval"
          type="range"
          min="50"
          max="2000"
          step="50"
          bind:value={textScrollInterval}
          disabled={!textConfigLoaded || !isEffectsOwner}
          class="mt-3 w-full cursor-pointer accent-blue-600"
        />

        <button
          type="button"
          onclick={saveTextScrollInterval}
          disabled={!effectsConnected || !isEffectsOwner ||
                    !textConfigLoaded || textSavePending}
          class="mt-3 w-full rounded-lg bg-green-600 px-6 py-3 font-semibold text-white disabled:opacity-50"
        >
          Save Scroll Speed
        </button>
      </div>

      <!-- Select the scrolling animation behavior -->
      <div class="mt-6">
        <label for="text-animation-style" class="block text-sm font-medium text-slate-300">
          Animation Style
        </label>

        <select
          id="text-animation-style"
          bind:value={textAnimationStyle}
          disabled={!textConfigLoaded || !isEffectsOwner}
          class="mt-3 w-full rounded-lg border border-slate-700 bg-slate-800 px-3 py-3 text-white"
        >
          <option value="standard">Standard Scrolling</option>
          <option value="weave" disabled>Vertical Weaving (Coming Soon)</option>
          <option value="moving_face" disabled>Moving Face (Coming Soon)</option>
        </select>

        <p class="mt-2 text-xs text-slate-400">
          Additional animation styles will become available when firmware support is implemented.
        </p>
      </div>

      <!-- Restore individual character colors -->
      <button
        type="button"
        onclick={resetTextColors}
        disabled={!effectsConnected || !isEffectsOwner ||
                  !textConfigLoaded || textSavePending}
        class="mt-6 w-full rounded-lg bg-slate-700 px-6 py-3 font-semibold text-white hover:bg-slate-600 disabled:opacity-50"
      >
        Reset Character Colors
      </button>

      <!-- Restore the message, colors, and scrolling interval -->
      <button
        type="button"
        onclick={resetTextConfig}
        disabled={!effectsConnected || !isEffectsOwner ||
                  !textConfigLoaded || textSavePending}
        class="mt-3 w-full rounded-lg bg-slate-700 px-6 py-3 font-semibold text-white hover:bg-slate-600 disabled:opacity-50"
      >
        Reset Text Configuration
      </button>

      <!-- Display firmware command results -->
      {#if textStatus}
        <p class="mt-3 text-center text-sm text-slate-300">
          {textStatus}
        </p>
      {/if}

      <!-- Activate or deactivate scrolling text -->
      <button
        type="button"
        onclick={toggleText}
        disabled={!effectsConnected || !isEffectsOwner ||
                  !textConfigLoaded || activeLedMode === 'unknown' ||
                  effectsTransitioning || textModePending}
        class="mt-8 w-full rounded-lg bg-blue-600 px-6 py-3 font-semibold text-white hover:bg-blue-500 disabled:opacity-50"
      >
        {activeLedMode === 'text' ? 'Deactivate Text' : 'Activate Text'}
      </button>

  {/if}
  </div>
  </div>
</main>