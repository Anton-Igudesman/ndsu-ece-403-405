<script>
    async function requestAccess() {
      try {
        const response = await fetch("/api/controller/request", {
          method: "POST"
        });

        const result = await response.json();

        console.log("Controller request:", result);
      } catch (error) {
        console.error("Controller request failed:", error);
      }
    }
</script>
<main class="min-h-screen flex items-center justify-center bg-slate-950">
  <div class="text-center">
    <h1 class="text-5xl font-bold text-white">
      LED Cube
    </h1>

    <p class="mt-4 text-lg text-slate-400">
      Control Interface
    </p>

    <button
      class="mt-8 rounded-lg bg-blue-600 px-6 py-3 font-semibold text-white hover:bg-blue-500"
      onclick={requestAccess}
    >
      Request Access
    </button>
  </div>
</main>