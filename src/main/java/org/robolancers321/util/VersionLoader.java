/* (C) Robolancers 2024 */
package org.robolancers321.util;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import org.yaml.snakeyaml.Yaml;

public class VersionLoader {

  public static class Versioning {

    public String title;
    public String user;
    public String updated_at;
    public boolean comp;

    public String toString() {
      return title + user + updated_at + comp;
    }
  }

  private final Versioning versionData;

  public VersionLoader() throws FileNotFoundException {
    InputStream inputStream =
        new FileInputStream(Filesystem.getDeployDirectory() + "/versioning.yml");

    Yaml yaml = new Yaml();

    this.versionData = yaml.load(inputStream);
  }

  public Versioning getVersionData() {
    return this.versionData;
  }
}
