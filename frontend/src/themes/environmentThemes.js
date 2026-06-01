/**
 * Environment Theme Definitions for eVTOL Simulation
 * Defines color palettes, lighting, and atmospheric properties for each theme
 */

export const THEMES = {
    DAYLIGHT: {
        id: 'daylight',
        name: 'Daylight',
        description: 'Clear daylight with bright skies and natural lighting',
        // Scene lighting
        sceneBackground: 0x87ceeb,           // Sky blue
        ambientLight: { color: 0xffffff, intensity: 0.28 },
        directionalLight: { color: 0xffffff, intensity: 1.8, position: [150, 250, 100] },
        hemisphereLight: { skyColor: 0xffffff, groundColor: 0x444444, intensity: 0.4 },
        fog: { color: 0x87ceeb, near: 100, far: 25000 },
        // UI Palette
        uiBackground: 'rgba(2, 6, 23, 0.8)',
        uiText: '#e2e8f0',
        uiAccent: '#3b82f6',
        uiAccentLight: '#60a5fa',
        buttonPrimary: '#3b82f6',
        buttonSecondary: '#10b981',
        panelBorder: 'rgba(255, 255, 255, 0.08)',
        telemetryBg: 'rgba(15, 23, 42, 0.7)',
        telemetryText: '#e2e8f0',
    },
    DAWN: {
        id: 'dawn',
        name: 'Dawn',
        description: 'Early morning with pink and violet hues',
        // Scene lighting
        sceneBackground: 0xcc99dd,           // Soft pink-purple
        ambientLight: { color: 0xffccee, intensity: 0.25 },
        directionalLight: { color: 0xffaa99, intensity: 1.5, position: [120, 180, 50] },
        hemisphereLight: { skyColor: 0xffccdd, groundColor: 0x664488, intensity: 0.35 },
        fog: { color: 0xcc99dd, near: 100, far: 22000 },
        // UI Palette
        uiBackground: 'rgba(20, 10, 25, 0.85)',
        uiText: '#fde2ff',
        uiAccent: '#d946ef',
        uiAccentLight: '#ec4899',
        buttonPrimary: '#d946ef',
        buttonSecondary: '#a855f7',
        panelBorder: 'rgba(219, 112, 147, 0.15)',
        telemetryBg: 'rgba(30, 10, 35, 0.8)',
        telemetryText: '#fde2ff',
    },
    SUNSET: {
        id: 'sunset',
        name: 'Sunset',
        description: 'Golden hour with warm orange and pink tones',
        // Scene lighting
        sceneBackground: 0xff9966,           // Orange-gold
        ambientLight: { color: 0xffcc99, intensity: 0.35 },
        directionalLight: { color: 0xffbb44, intensity: 2.0, position: [180, 200, 80] },
        hemisphereLight: { skyColor: 0xffbb44, groundColor: 0xbb6633, intensity: 0.45 },
        fog: { color: 0xff9966, near: 100, far: 20000 },
        // UI Palette
        uiBackground: 'rgba(20, 12, 5, 0.85)',
        uiText: '#fde047',
        uiAccent: '#f97316',
        uiAccentLight: '#fb923c',
        buttonPrimary: '#f97316',
        buttonSecondary: '#d97706',
        panelBorder: 'rgba(255, 153, 102, 0.15)',
        telemetryBg: 'rgba(30, 18, 8, 0.8)',
        telemetryText: '#fde047',
    },
    DUSK: {
        id: 'dusk',
        name: 'Dusk',
        description: 'Evening twilight with deep purple and indigo',
        // Scene lighting
        sceneBackground: 0x4a3480,           // Deep purple
        ambientLight: { color: 0x7755ff, intensity: 0.22 },
        directionalLight: { color: 0x9966ff, intensity: 1.2, position: [-150, 160, 120] },
        hemisphereLight: { skyColor: 0x6655ff, groundColor: 0x332255, intensity: 0.3 },
        fog: { color: 0x4a3480, near: 100, far: 18000 },
        // UI Palette
        uiBackground: 'rgba(15, 8, 25, 0.9)',
        uiText: '#e9d5ff',
        uiAccent: '#a78bfa',
        uiAccentLight: '#c4b5fd',
        buttonPrimary: '#a78bfa',
        buttonSecondary: '#8b5cf6',
        panelBorder: 'rgba(167, 139, 250, 0.12)',
        telemetryBg: 'rgba(20, 10, 35, 0.85)',
        telemetryText: '#e9d5ff',
    },
    MOONLIGHT: {
        id: 'moonlight',
        name: 'Moonlight',
        description: 'Night mode with moon and stars',
        // Scene lighting
        sceneBackground: 0x1a1a2e,           // Deep navy
        ambientLight: { color: 0x7799ff, intensity: 0.15 },
        directionalLight: { color: 0xbbddff, intensity: 0.9, position: [-180, 300, 150] },
        hemisphereLight: { skyColor: 0x3366ff, groundColor: 0x1a1a2e, intensity: 0.2 },
        fog: { color: 0x1a1a2e, near: 100, far: 15000 },
        // UI Palette
        uiBackground: 'rgba(10, 10, 20, 0.9)',
        uiText: '#e0f2fe',
        uiAccent: '#0ea5e9',
        uiAccentLight: '#38bdf8',
        buttonPrimary: '#0ea5e9',
        buttonSecondary: '#6366f1',
        panelBorder: 'rgba(14, 165, 233, 0.1)',
        telemetryBg: 'rgba(15, 23, 42, 0.85)',
        telemetryText: '#e0f2fe',
    },
    OVERCAST: {
        id: 'overcast',
        name: 'Overcast',
        description: 'Cloudy day with muted gray tones',
        // Scene lighting
        sceneBackground: 0x9199a8,           // Gray-blue
        ambientLight: { color: 0xccccdd, intensity: 0.45 },
        directionalLight: { color: 0xbbbbcc, intensity: 1.2, position: [100, 150, 80] },
        hemisphereLight: { skyColor: 0xbbbbdd, groundColor: 0x666677, intensity: 0.5 },
        fog: { color: 0x9199a8, near: 100, far: 16000 },
        // UI Palette
        uiBackground: 'rgba(15, 15, 20, 0.85)',
        uiText: '#e5e7eb',
        uiAccent: '#6b7280',
        uiAccentLight: '#9ca3af',
        buttonPrimary: '#6b7280',
        buttonSecondary: '#4b5563',
        panelBorder: 'rgba(200, 200, 220, 0.08)',
        telemetryBg: 'rgba(25, 25, 35, 0.8)',
        telemetryText: '#e5e7eb',
    },
    AURORA: {
        id: 'aurora',
        name: 'Aurora',
        description: 'Northern lights with vibrant greens and blues',
        // Scene lighting
        sceneBackground: 0x0d1b2a,           // Very dark blue
        ambientLight: { color: 0x00ff88, intensity: 0.2 },
        directionalLight: { color: 0x00ffcc, intensity: 1.1, position: [140, 200, 100] },
        hemisphereLight: { skyColor: 0x00ff88, groundColor: 0x003322, intensity: 0.28 },
        fog: { color: 0x0d1b2a, near: 100, far: 19000 },
        // UI Palette
        uiBackground: 'rgba(5, 15, 25, 0.9)',
        uiText: '#a0f7c3',
        uiAccent: '#06d6a0',
        uiAccentLight: '#38efc8',
        buttonPrimary: '#06d6a0',
        buttonSecondary: '#00d99f',
        panelBorder: 'rgba(6, 214, 160, 0.12)',
        telemetryBg: 'rgba(10, 20, 30, 0.85)',
        telemetryText: '#a0f7c3',
    },
};

export const getTheme = (themeId) => {
    return THEMES[Object.keys(THEMES).find(key => THEMES[key].id === themeId)] || THEMES.DAYLIGHT;
};

export const getThemeList = () => {
    return Object.values(THEMES).map(theme => ({
        id: theme.id,
        name: theme.name,
        description: theme.description,
    }));
};
