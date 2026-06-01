import React from "react";
import { getThemeList } from "../themes/environmentThemes";
import { useState, useEffect } from "react";

// Add custom scrollbar styles
const scrollbarStyles = `
  .theme-selector-dropdown::-webkit-scrollbar {
    width: 8px;
  }
  .theme-selector-dropdown::-webkit-scrollbar-track {
    background: rgba(255, 255, 255, 0.05);
  }
  .theme-selector-dropdown::-webkit-scrollbar-thumb {
    background: rgba(255, 255, 255, 0.2);
    border-radius: 4px;
  }
  .theme-selector-dropdown::-webkit-scrollbar-thumb:hover {
    background: rgba(255, 255, 255, 0.3);
  }
  .theme-selector-dropdown {
    scrollbar-color: rgba(255, 255, 255, 0.2) rgba(255, 255, 255, 0.05);
    scrollbar-width: thin;
  }
`;

export default function ThemeSelector({ currentTheme, onThemeChange }) {
    const [showDropdown, setShowDropdown] = useState(false);
    const themes = getThemeList();
    const activeTheme = themes.find(t => t.id === currentTheme) || themes[0];

    useEffect(() => {
        // Inject scrollbar styles
        if (!document.getElementById('theme-selector-scrollbar-styles')) {
            const style = document.createElement('style');
            style.id = 'theme-selector-scrollbar-styles';
            style.innerHTML = scrollbarStyles;
            document.head.appendChild(style);
        }
    }, []);

    return (
        <div style={styles.container}>
            <div style={styles.selectorWrapper}>
                <button
                    onClick={() => setShowDropdown(!showDropdown)}
                    style={{
                        ...styles.selectorButton,
                        borderColor: getThemeBorderColor(currentTheme),
                    }}
                    title="Switch environment theme"
                >
                    <span style={styles.icon}>🌍</span>
                    <span>{activeTheme.name}</span>
                    <span style={{ marginLeft: '8px', fontSize: '12px' }}>
                        {showDropdown ? '▲' : '▼'}
                    </span>
                </button>

                {showDropdown && (
                    <div style={styles.dropdown} className="theme-selector-dropdown">
                        {themes.map((theme) => (
                            <button
                                key={theme.id}
                                onClick={() => {
                                    onThemeChange(theme.id);
                                    setShowDropdown(false);
                                }}
                                style={{
                                    ...styles.dropdownItem,
                                    backgroundColor:
                                        currentTheme === theme.id
                                            ? getThemeAccentColor(theme.id)
                                            : 'transparent',
                                    color:
                                        currentTheme === theme.id
                                            ? getThemeTextColor(theme.id)
                                            : '#a8b2c1',
                                }}
                            >
                                <span style={{ fontSize: '16px', marginRight: '8px' }}>
                                    {getThemeIcon(theme.id)}
                                </span>
                                <div style={{ textAlign: 'left' }}>
                                    <div style={{ fontWeight: 600, fontSize: '14px' }}>
                                        {theme.name}
                                    </div>
                                    <div style={{ fontSize: '11px', opacity: 0.8 }}>
                                        {theme.description}
                                    </div>
                                </div>
                            </button>
                        ))}
                    </div>
                )}
            </div>
        </div>
    );
}

function getThemeIcon(themeId) {
    switch (themeId) {
        case 'daylight':
            return '☀️';
        case 'dawn':
            return '🌅';
        case 'sunset':
            return '🌅';
        case 'dusk':
            return '🌆';
        case 'moonlight':
            return '🌙';
        case 'overcast':
            return '☁️';
        case 'aurora':
            return '🌌';
        default:
            return '🌍';
    }
}

function getThemeBorderColor(themeId) {
    switch (themeId) {
        case 'daylight':
            return '#60a5fa';
        case 'dawn':
            return '#ec4899';
        case 'sunset':
            return '#fb923c';
        case 'dusk':
            return '#c4b5fd';
        case 'moonlight':
            return '#38bdf8';
        case 'overcast':
            return '#9ca3af';
        case 'aurora':
            return '#38efc8';
        default:
            return '#60a5fa';
    }
}

function getThemeAccentColor(themeId) {
    switch (themeId) {
        case 'daylight':
            return 'rgba(59, 130, 246, 0.2)';
        case 'dawn':
            return 'rgba(217, 70, 239, 0.2)';
        case 'sunset':
            return 'rgba(249, 115, 22, 0.2)';
        case 'dusk':
            return 'rgba(167, 139, 250, 0.2)';
        case 'moonlight':
            return 'rgba(14, 165, 233, 0.2)';
        case 'overcast':
            return 'rgba(107, 114, 128, 0.2)';
        case 'aurora':
            return 'rgba(6, 214, 160, 0.2)';
        default:
            return 'rgba(59, 130, 246, 0.2)';
    }
}

function getThemeTextColor(themeId) {
    switch (themeId) {
        case 'daylight':
            return '#e2e8f0';
        case 'dawn':
            return '#fde2ff';
        case 'sunset':
            return '#fde047';
        case 'dusk':
            return '#e9d5ff';
        case 'moonlight':
            return '#e0f2fe';
        case 'overcast':
            return '#e5e7eb';
        case 'aurora':
            return '#a0f7c3';
        default:
            return '#e2e8f0';
    }
}

const styles = {
    container: {
        padding: '8px 0',
        position: 'relative',
    },
    selectorWrapper: {
        position: 'relative',
        display: 'inline-block',
    },
    selectorButton: {
        display: 'flex',
        alignItems: 'center',
        gap: '8px',
        padding: '8px 16px',
        background: 'rgba(2, 6, 23, 0.6)',
        border: '1px solid #60a5fa',
        borderRadius: '6px',
        color: '#e2e8f0',
        fontFamily: 'Orbitron, sans-serif',
        fontSize: '13px',
        fontWeight: 600,
        cursor: 'pointer',
        transition: 'all 0.3s ease',
        backdropFilter: 'blur(8px)',
    },
    icon: {
        fontSize: '16px',
    },
    dropdown: {
        position: 'absolute',
        top: '100%',
        left: 0,
        right: 0,
        marginTop: '8px',
        background: 'rgba(15, 23, 42, 0.95)',
        border: '1px solid rgba(255, 255, 255, 0.1)',
        borderRadius: '8px',
        backdropFilter: 'blur(12px)',
        boxShadow: '0 10px 30px rgba(0, 0, 0, 0.5)',
        zIndex: 1000,
        overflow: "auto",
        maxHeight: "320px",
    },
    dropdownItem: {
        display: 'flex',
        alignItems: 'center',
        width: '100%',
        padding: '12px 16px',
        background: 'transparent',
        border: 'none',
        borderBottom: '1px solid rgba(255, 255, 255, 0.05)',
        color: '#a8b2c1',
        fontFamily: 'Orbitron, sans-serif',
        cursor: 'pointer',
        transition: 'all 0.2s ease',
        textAlign: 'left',
    },
};
