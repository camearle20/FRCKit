package frckit.simulation.modelconfig;

import com.google.gson.Gson;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;

public class ConfigurationLoader {
    private static final Gson gson = new Gson();

    public static ModelConfiguration load() throws FileNotFoundException {
        System.out.println("Looking for configuration file at '" + (new File("model.json").getAbsolutePath()) + "'");
        return gson.fromJson(new FileReader("model.json"), ModelConfiguration.class);
    }
}
