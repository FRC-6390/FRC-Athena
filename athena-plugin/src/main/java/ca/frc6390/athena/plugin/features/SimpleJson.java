package ca.frc6390.athena.plugin.features;

import java.io.IOException;
import java.io.Reader;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Small strict JSON reader for Athena plugin metadata resources.
 */
final class SimpleJson {
    private final String source;
    private final String text;
    private int index;

    private SimpleJson(String text, String source) {
        this.text = text;
        this.source = source;
    }

    static Object parse(Reader reader, String source) {
        StringBuilder text = new StringBuilder();
        char[] buffer = new char[1024];
        try {
            int read;
            while ((read = reader.read(buffer)) != -1) {
                text.append(buffer, 0, read);
            }
        } catch (IOException exception) {
            throw new VendorMetadataException("Failed to read JSON from " + source + ".", exception);
        }
        SimpleJson parser = new SimpleJson(text.toString(), source);
        Object value = parser.value();
        parser.skipWhitespace();
        if (!parser.finished()) {
            parser.fail("unexpected trailing content");
        }
        return value;
    }

    private Object value() {
        skipWhitespace();
        if (finished()) {
            fail("unexpected end of input");
        }
        return switch (peek()) {
            case '{' -> object();
            case '[' -> array();
            case '"' -> string();
            case 't' -> literal("true", Boolean.TRUE);
            case 'f' -> literal("false", Boolean.FALSE);
            case 'n' -> literal("null", null);
            default -> fail("expected JSON value");
        };
    }

    private Map<String, Object> object() {
        expect('{');
        Map<String, Object> object = new LinkedHashMap<>();
        skipWhitespace();
        if (consume('}')) {
            return object;
        }
        do {
            skipWhitespace();
            if (finished() || peek() != '"') {
                fail("expected object key");
            }
            String key = string();
            skipWhitespace();
            expect(':');
            object.put(key, value());
            skipWhitespace();
        } while (consume(','));
        expect('}');
        return object;
    }

    private List<Object> array() {
        expect('[');
        List<Object> array = new ArrayList<>();
        skipWhitespace();
        if (consume(']')) {
            return array;
        }
        do {
            array.add(value());
            skipWhitespace();
        } while (consume(','));
        expect(']');
        return array;
    }

    private String string() {
        expect('"');
        StringBuilder result = new StringBuilder();
        while (!finished()) {
            char current = next();
            if (current == '"') {
                return result.toString();
            }
            if (current == '\\') {
                result.append(escape());
            } else {
                result.append(current);
            }
        }
        return fail("unterminated string");
    }

    private char escape() {
        if (finished()) {
            fail("unterminated escape sequence");
        }
        return switch (next()) {
            case '"' -> '"';
            case '\\' -> '\\';
            case '/' -> '/';
            case 'b' -> '\b';
            case 'f' -> '\f';
            case 'n' -> '\n';
            case 'r' -> '\r';
            case 't' -> '\t';
            case 'u' -> unicode();
            default -> fail("invalid escape sequence");
        };
    }

    private char unicode() {
        if (index + 4 > text.length()) {
            fail("incomplete unicode escape");
        }
        String digits = text.substring(index, index + 4);
        try {
            index += 4;
            return (char) Integer.parseInt(digits, 16);
        } catch (NumberFormatException exception) {
            throw error("invalid unicode escape");
        }
    }

    private Object literal(String literal, Object value) {
        if (!text.startsWith(literal, index)) {
            fail("expected " + literal);
        }
        index += literal.length();
        return value;
    }

    private void skipWhitespace() {
        while (!finished() && Character.isWhitespace(peek())) {
            index++;
        }
    }

    private boolean consume(char expected) {
        if (!finished() && peek() == expected) {
            index++;
            return true;
        }
        return false;
    }

    private void expect(char expected) {
        if (finished() || next() != expected) {
            fail("expected '" + expected + "'");
        }
    }

    private char peek() {
        return text.charAt(index);
    }

    private char next() {
        return text.charAt(index++);
    }

    private boolean finished() {
        return index >= text.length();
    }

    private <T> T fail(String message) {
        throw error(message);
    }

    private VendorMetadataException error(String message) {
        return new VendorMetadataException(source + " contains invalid JSON at offset " + index + ": " + message + ".");
    }
}
