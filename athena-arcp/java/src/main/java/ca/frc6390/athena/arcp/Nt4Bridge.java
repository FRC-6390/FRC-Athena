package ca.frc6390.athena.arcp;

import java.util.List;

/**
 * Hook for mirroring selected ARCP subtrees to NetworkTables 4 (E7). The default implementation
 * does nothing; a real bridge can publish the dictionary and changed values under NT topics.
 */
public interface Nt4Bridge {

    /** Bridge that ignores everything. */
    Nt4Bridge DISABLED = new Nt4Bridge() {
        @Override
        public void dictionaryChanged(List<ArcpDictEntry> dictionary) {
        }

        @Override
        public void valueChanged(ArcpDictEntry entry, Object value) {
        }
    };

    /** Called after the dictionary is replaced. */
    void dictionaryChanged(List<ArcpDictEntry> dictionary);

    /** Called for each signal value that changed in a published cycle. */
    void valueChanged(ArcpDictEntry entry, Object value);
}
