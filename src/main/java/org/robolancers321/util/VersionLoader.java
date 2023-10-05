/* (C) Robolancers 2024 */
package org.robolancers321.util;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import org.yaml.snakeyaml.Yaml;
import org.yaml.snakeyaml.constructor.Constructor;

public class VersionLoader {

  public static class Versioning {

    private String title;
    private String user;
    private String updated_at;
    private boolean comp;

    public String toString() {
      return title + user + updated_at + comp;
    }
  }

  private final Versioning versionData;

  public VersionLoader() throws FileNotFoundException {
    InputStream inputStream =
        new FileInputStream(Filesystem.getDeployDirectory() + "/versioning.yml");

    Yaml yaml = new Yaml(new Constructor(Versioning.class));

    this.versionData = yaml.load(inputStream);
  }

  public Versioning getVersionData() {
    return this.versionData;
  }
}
