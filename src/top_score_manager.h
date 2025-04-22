#pragma once

#include <string>

class TopScoreManager {
public:
  /// Construct with the filename to use for persistence.
  explicit TopScoreManager(const std::string& filename);

  /// Load the top score from disk. Returns 0 if the file doesn't exist or is invalid.
  int LoadTopScore();

  /**
   *  If score > previously loaded top score, overwrite the file and return true.
   *  Otherwise do nothing and return false.
   */
  bool SaveTopScore(int score);

private:
  std::string _filename;
  int _cachedTopScore = 0;
};
