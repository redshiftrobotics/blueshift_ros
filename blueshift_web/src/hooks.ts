// This disables SSR: https://github.com/sveltejs/kit/discussions/3365#discussioncomment-2740818
// See https://kit.svelte.dev/docs/hooks#handle for more details

import type { Handle } from "@sveltejs/kit";

export const handle: Handle = async ({ event, resolve }) => {
    const response = await resolve(event, {
        ssr: false,
    });
    return response;
};