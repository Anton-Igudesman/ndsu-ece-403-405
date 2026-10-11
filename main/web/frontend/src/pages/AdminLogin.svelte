<script>
  let username = ''
  let password = ''
  let errorMessage = ''
  let loggingIn = false

  async function handleLogin() {
    errorMessage = ''
    loggingIn = true

    try {
      const response = await fetch('/api/admin/login', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          username,
          password
        })
      })

      if (!response.ok) {
        errorMessage = 'Invalid username or password'
        return
      }

      window.location.hash = '/admin/dashboard'
    } catch {
      errorMessage = 'Unable to reach the LED Cube'
    } finally {
      loggingIn = false
    }
  }
</script>

<main class="min-h-screen flex items-center justify-center bg-slate-950 px-6">
  <div class="w-full max-w-md">
    <div class="mb-8 text-center">
      <h1 class="text-3xl font-bold text-white">
        LED Cube Admin
      </h1>

      <p class="mt-2 text-slate-400">
        Administrator access
      </p>
    </div>

    <form
      class="rounded-xl border border-slate-800 bg-slate-900 p-8"
      on:submit|preventDefault={handleLogin}
    >
      <label
        for="username"
        class="block text-sm font-medium text-slate-300"
      >
        Username
      </label>

      <input
        id="username"
        type="text"
        bind:value={username}
        autocomplete="username"
        required
        class="mt-2 w-full rounded-lg border border-slate-700 bg-slate-950 px-4 py-3 text-white outline-none focus:border-blue-500"
      />

      <label
        for="password"
        class="mt-6 block text-sm font-medium text-slate-300"
      >
        Password
      </label>

      <input
        id="password"
        type="password"
        bind:value={password}
        autocomplete="current-password"
        required
        class="mt-2 w-full rounded-lg border border-slate-700 bg-slate-950 px-4 py-3 text-white outline-none focus:border-blue-500"
      />

      {#if errorMessage}
        <p class="mt-4 text-sm text-red-400">
          {errorMessage}
        </p>
      {/if}

      <button
        type="submit"
        disabled={loggingIn}
        class="mt-8 w-full rounded-lg bg-blue-600 px-4 py-3 font-semibold text-white hover:bg-blue-500 disabled:cursor-not-allowed disabled:opacity-50"
      >
        {loggingIn ? 'Signing in...' : 'Sign In'}
      </button>
    </form>

    <a
      href="#/"
      class="mt-6 block text-center text-sm text-slate-400 hover:text-white"
    >
      Return to LED Cube
    </a>
  </div>
</main>