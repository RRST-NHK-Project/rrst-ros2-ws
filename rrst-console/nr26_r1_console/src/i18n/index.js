import { runtimeTextByJa, uiTextByJaByLanguage } from "./dictionary";

export const SUPPORTED_LANGUAGES = [
    "ja",
    "en",
    "de",
    "fr",
    "es",
    "zh",
    "pt",
    "it",
    "ru",
    "ko",
    "ar",
    "hi",
    "id",
    "tr",
];

export const LANGUAGE_OPTIONS = [
    { code: "ja", label: "日本語", flag: "🇯🇵" },
    { code: "en", label: "English", flag: "🇺🇸" },
    { code: "de", label: "Deutsch", flag: "🇩🇪" },
    { code: "fr", label: "Francais", flag: "🇫🇷" },
    { code: "es", label: "Espanol", flag: "🇪🇸" },
    { code: "zh", label: "中文", flag: "🇨🇳" },
    { code: "pt", label: "Portugues", flag: "🇧🇷" },
    { code: "it", label: "Italiano", flag: "🇮🇹" },
    { code: "ru", label: "Русский", flag: "🇷🇺" },
    { code: "ko", label: "한국어", flag: "🇰🇷" },
    { code: "ar", label: "العربية", flag: "🇸🇦" },
    { code: "hi", label: "हिन्दी", flag: "🇮🇳" },
    { code: "id", label: "Bahasa Indonesia", flag: "🇮🇩" },
    { code: "tr", label: "Turkce", flag: "🇹🇷" },
];

export const getLocalizedText = (language, jaText, enText) => {
    if (language === "ja") {
        return jaText;
    }
    if (language === "en") {
        return enText;
    }

    const langMap = uiTextByJaByLanguage[language];
    if (langMap) {
        return langMap[jaText] || enText;
    }

    return enText;
};

export const translateRuntimeText = (language, text) => {
    if (!text || language === "ja") {
        return text;
    }

    const monitorMatch = text.match(/^(.+) \((.+)\) を監視中$/);
    if (monitorMatch) {
        if (language === "de") {
            return `Ueberwache ${monitorMatch[1]} (${monitorMatch[2]})`;
        }
        if (language === "fr") {
            return `Surveillance ${monitorMatch[1]} (${monitorMatch[2]})`;
        }
        if (language === "es") {
            return `Monitoreando ${monitorMatch[1]} (${monitorMatch[2]})`;
        }
        if (language === "zh") {
            return `监视中 ${monitorMatch[1]} (${monitorMatch[2]})`;
        }
        return `Monitoring ${monitorMatch[1]} (${monitorMatch[2]})`;
    }

    const serialTxMatch = text.match(/^(.+) に (\d+) 要素を送信 \(入力 (\d+) 要素\)$/);
    if (serialTxMatch) {
        if (language === "de") {
            return `${serialTxMatch[2]} Elemente an ${serialTxMatch[1]} gesendet (Eingang ${serialTxMatch[3]})`;
        }
        if (language === "fr") {
            return `${serialTxMatch[2]} elements envoyes a ${serialTxMatch[1]} (entree ${serialTxMatch[3]})`;
        }
        if (language === "es") {
            return `${serialTxMatch[2]} elementos enviados a ${serialTxMatch[1]} (entrada ${serialTxMatch[3]})`;
        }
        if (language === "zh") {
            return `已向 ${serialTxMatch[1]} 发送 ${serialTxMatch[2]} 个元素 (输入 ${serialTxMatch[3]})`;
        }
        return `Sent ${serialTxMatch[2]} elements to ${serialTxMatch[1]} (input ${serialTxMatch[3]})`;
    }

    const periodicMatch = text.match(/^定期送信中: (.+) Hz$/);
    if (periodicMatch) {
        if (language === "de") {
            return `Periodisches Senden: ${periodicMatch[1]} Hz`;
        }
        if (language === "fr") {
            return `Envoi periodique: ${periodicMatch[1]} Hz`;
        }
        if (language === "es") {
            return `Envio periodico: ${periodicMatch[1]} Hz`;
        }
        if (language === "zh") {
            return `周期发送: ${periodicMatch[1]} Hz`;
        }
        return `Periodic sending: ${periodicMatch[1]} Hz`;
    }

    const savePoseMatch = text.match(/^目標値を保存しました: \((.+)\)$/);
    if (savePoseMatch) {
        if (language === "de") {
            return `Ziel-Pose gespeichert: (${savePoseMatch[1]})`;
        }
        if (language === "fr") {
            return `Pose cible enregistree: (${savePoseMatch[1]})`;
        }
        if (language === "es") {
            return `Pose objetivo guardada: (${savePoseMatch[1]})`;
        }
        if (language === "zh") {
            return `已保存目标位姿: (${savePoseMatch[1]})`;
        }
        return `Saved target pose: (${savePoseMatch[1]})`;
    }

    const savedSendMatch = text.match(/^保存値 "(.+)" を送信しました$/);
    if (savedSendMatch) {
        if (language === "de") {
            return `Gespeichertes Ziel "${savedSendMatch[1]}" gesendet`;
        }
        if (language === "fr") {
            return `Valeur enregistree "${savedSendMatch[1]}" envoyee`;
        }
        if (language === "es") {
            return `Valor guardado "${savedSendMatch[1]}" enviado`;
        }
        if (language === "zh") {
            return `已发送保存值 "${savedSendMatch[1]}"`;
        }
        return `Sent saved target "${savedSendMatch[1]}"`;
    }

    const hit = runtimeTextByJa[text];
    if (!hit) {
        return text;
    }

    return hit[language] || hit.en || text;
};
